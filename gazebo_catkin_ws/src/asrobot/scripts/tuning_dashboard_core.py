"""独立参数校验、时间序列清理及响应指标，不依赖ROS或GUI。"""
import math
import statistics

# 标签、单位、步长和分组；范围匹配当前控制器正参数约束，C++ valid()做最终裁决。
PARAMETERS = {
 'target_z': ('标记 Z 目标', 'm', -.50, -.141, .005, '接近与保持'),
 'max_speed': ('最大接近速度', 'm/s', .001, .08, .001, '接近与保持'),
 'near_speed': ('末段速度', 'm/s', .001, .04, .001, '接近与保持'),
 'near_distance': ('末段误差范围', 'm', .01, .30, .01, '接近与保持'),
 'acceleration': ('目标加速度上限', 'm/s²', .001, .10, .001, '接近与保持'),
 'position_kp': ('Z 位置外环 Kp', '1/s', .001, 3., .01, '垂直双环'),
 'velocity_kp': ('Z 速度内环 Kp', '', .01, 60., .1, '垂直双环'),
 'velocity_ki': ('Z 速度内环 Ki', '', .001, 10., .01, '垂直双环'),
 'z_force_limit': ('垂直力上限', 'N', .05, 2., .05, '垂直双环'),
 'xy_kp': ('XY 位置 Kp', '', .001, 5., .01, 'XY 与航向'),
 'xy_kd': ('XY 速度阻尼', '', .001, 5., .01, 'XY 与航向'),
 'xy_force_limit': ('XY 力上限', 'N', .02, .8, .01, 'XY 与航向'),
 'yaw_kp': ('航向 Kp', 'Nm/°', .00001, .05, .0005, 'XY 与航向'),
 'yaw_kd': ('角速度阻尼', '', .0001, .5, .001, 'XY 与航向'),
 'yaw_limit': ('偏航力矩上限', 'Nm', .001, .10, .001, 'XY 与航向'),
 'xy_tolerance': ('远处 XY 允许误差', 'm', .005, .05, .001, '到位判定'),
 'xy_hysteresis': ('XY 退出回差', 'm', .001, .03, .001, '到位判定'),
 'z_tolerance': ('高度允许误差', 'm', .001, .03, .001, '到位判定'),
 'velocity_tolerance': ('到位速度阈值', 'm/s', .001, .03, .001, '到位判定'),
 'yaw_tolerance': ('到位航向误差', '°', .1, 15., .1, '到位判定'),
 'settle_time': ('到位稳定确认', 's', .2, 5., .1, '到位判定'),
 'timeout': ('测量有效期', 's', .1, .5, .05, '失效与恢复'),
 'wait_timeout': ('首帧等待时间', 's', 1.1, 10., .1, '失效与恢复'),
 'recovery_timeout': ('视觉重获时限', 's', .5, 5., .1, '失效与恢复'),
 'transport_timeout': ('通信等待时限', 's', .5, 10., .1, '失效与恢复'),
 'reacquire_frames': ('重获连续新帧', '帧', 1, 15, 1, '失效与恢复'),
}


def validate_config(raw):
    """严格拒绝未知字段、bool、NaN和非法组合，不能静默截断帧数。"""
    if not isinstance(raw, dict) or set(raw) != set(PARAMETERS):
        raise ValueError('参数字段不完整或包含未知字段，请先读取节点参数')
    config = {}
    for key, (_, _, lo, hi, _, _) in PARAMETERS.items():
        value = raw[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(key + ' 必须是有限数字')
        if value < lo or value > hi:
            raise ValueError('{} 必须在 {}～{} 之间'.format(key, lo, hi))
        if key == 'reacquire_frames':
            if int(value) != value:
                raise ValueError('重获帧数必须为整数')
            value = int(value)
        config[key] = value
    if config['near_speed'] > config['max_speed']:
        raise ValueError('末段速度不能大于最大接近速度')
    if config['timeout'] > config['wait_timeout'] or config['timeout'] > config['recovery_timeout']:
        raise ValueError('等待和恢复时限不能小于数据有效期')
    if config['transport_timeout'] < config['recovery_timeout']:
        raise ValueError('通信等待时限不能小于视觉重获时限')
    return config


def schema():
    return [dict(key=k, label=v[0], unit=v[1], min=v[2], max=v[3], step=v[4], group=v[5])
            for k, v in PARAMETERS.items()]


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def response_metrics(samples, target, tolerance, dwell=1., max_gap=.6, complete=False):
    """方向无关阶跃指标。输入包含t/value/valid；重复时间丢弃，数据缺口不插值造响应。

    超调=max(direction*(y-target),0)，除以|target-initial|得到百分比。
    上升时间=首次10%至90%的线性插值时间，仅在连续有效采样之间插值。
    调节时间=末次超出误差带/缺口之后进入并持续dwell的尾段；实时结果标记暂定。
    """
    if not finite(target) or not finite(tolerance) or tolerance <= 0:
        raise ValueError('目标及误差带无效')
    points = []
    dropped = 0
    previous = None
    seen_bad = False
    for sample in samples:
        t, value = sample.get('t'), sample.get('value')
        if not finite(t) or not finite(value) or not sample.get('valid', True):
            seen_bad = True
            dropped += 1
            continue
        if previous is not None and t <= previous:
            dropped += 1
            continue
        gap = seen_bad or (previous is not None and t - previous > max_gap)
        points.append((float(t), float(value), gap))
        seen_bad = False
        previous = t
    result = dict(n=len(points), dropped=dropped, target=target, tolerance=tolerance,
                  overshoot=None, overshoot_pct=None, rise_time=None, peak_time=None,
                  settling_time=None, steady_error=None, rmse=None, duration=0,
                  status='等待有效数据', provisional=not complete, quality='无数据')
    if not points:
        return result
    t0, y0, _ = points[0]
    amplitude = abs(target-y0)
    direction = 1 if target >= y0 else -1
    result['duration'] = points[-1][0]-t0
    errors = [target-y for _, y, _ in points]
    result['rmse'] = math.sqrt(sum(e*e for e in errors)/len(errors))
    tail = [target-y for t,y,_ in points if t >= points[-1][0]-min(2., max(.5, result['duration']*.2))]
    result['steady_error'] = statistics.mean(tail)
    gaps = any(g for _,_,g in points[1:]) or seen_bad
    result['quality'] = '有缺口，指标仅供参考' if gaps else '连续有效采样'
    if amplitude > max(1e-6, tolerance*.1):
        excursions = [max(0., direction*(y-target)) for _,y,_ in points]
        over = max(excursions)
        result['overshoot'] = over
        result['overshoot_pct'] = 100.*over/amplitude
        if over > 0:
            idx = excursions.index(over)
            result['peak_time'] = points[idx][0]-t0
        crossing = {}
        for threshold in (.1, .9):
            for a,b in zip(points, points[1:]):
                pa = direction*(a[1]-y0)/amplitude
                pb = direction*(b[1]-y0)/amplitude
                if not b[2] and pa < threshold <= pb:
                    crossing[threshold] = a[0] + (threshold-pa)/(pb-pa)*(b[0]-a[0])
                    break
        if .1 in crossing and .9 in crossing:
            result['rise_time'] = crossing[.9]-crossing[.1]
    # 从数据尾部倒查“连续在带内”的起点；后续再次超差会撤销此前暂定结果。
    start = None
    for idx in range(len(points)-1, -1, -1):
        t,y,gap = points[idx]
        if abs(target-y) > tolerance:
            break
        start = idx
        if gap:
            break
    if start is not None and not seen_bad and points[-1][0]-points[start][0] >= dwell:
        result['settling_time'] = points[start][0]-t0
        result['status'] = '已进入误差带' if complete else '暂定稳定，等待继续观察'
    elif len(points) == 1:
        result['status'] = '采样中'
    else:
        result['status'] = '尚未稳定'
    return result


def series_for(rows, axis):
    """使用实际源时间去重，暂停/重复发布不能制造更多有效采样。"""
    result = []
    for row in rows:
        telemetry = row.get('telemetry', row)
        if axis == 'distance':
            value = telemetry.get('distance'); stamp = telemetry.get('source_t')
        elif axis in ('x', 'y'):
            value = telemetry.get(axis); stamp = telemetry.get('source_t')
        elif axis == 'yaw':
            value = -(telemetry.get('yaw_error') or 0) if finite(telemetry.get('yaw_error')) else None
            stamp = telemetry.get('feedback_t')
        else:
            raise ValueError('未知响应通道')
        result.append(dict(t=stamp, value=value, valid=bool(telemetry.get('valid'))))
    return result

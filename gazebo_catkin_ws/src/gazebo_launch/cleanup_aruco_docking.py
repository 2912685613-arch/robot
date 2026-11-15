#!/usr/bin/env python3
"""按运行目录清理包括独立会话在内的后代进程，不按程序名称全局杀进程。"""
import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import signal
import sys
import time


@dataclass(frozen=True)
class Process:
    """start_time 用来防止 PID 复用后误杀另一个进程。"""
    pid: int
    parent: int
    start_time: str
    command: str


def inspect(pid, proc_root=Path('/proc')):
    """读取内核进程身份；进程退出或无权限时跳过，不推测其身份。"""
    try:
        stat = (proc_root / str(pid) / 'stat').read_text()
        fields = stat[stat.rfind(')') + 2:].split()
        if fields[0] == 'Z':
            return None
        command = (proc_root / str(pid) / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
        return Process(int(pid), int(fields[1]), fields[19], command)
    except (OSError, ValueError, IndexError):
        return None


def ancestors(proc_root=Path('/proc')):
    """调用者及其祖先进程不能被清理器终止。"""
    excluded = set()
    pid = os.getpid()
    while pid and pid not in excluded:
        excluded.add(pid)
        item = inspect(pid, proc_root)
        if item is None:
            break
        pid = item.parent
    return excluded


def matching_processes(run_dir, proc_root=Path('/proc')):
    """用继承的环境标记/精确日志路径找本次进程；兼容旧脚本留下的独立会话。"""
    result = []
    excluded = ancestors(proc_root)
    token = ('ARUCO_DOCKING_SESSION=' + str(run_dir)).encode()
    ros_dir = ('ROS_LOG_DIR=' + str(run_dir / 'ros')).encode()
    log_arg = '__log:=' + str(run_dir / 'ros') + '/'
    for entry in proc_root.iterdir():
        if not entry.name.isdigit() or int(entry.name) in excluded:
            continue
        item = inspect(int(entry.name), proc_root)
        if item is None:
            continue
        try:
            environment = (entry / 'environ').read_bytes().split(b'\0')
        except OSError:
            environment = []
        if token in environment or ros_dir in environment or any(
                arg.startswith(log_arg) for arg in item.command.split()):
            result.append(item)
    return result


def signal_process(item, signum, proc_root=Path('/proc')):
    """再次核对 start_time 后发信号，防止清理期间 PID 被其他程序复用。"""
    current = inspect(item.pid, proc_root)
    if current is None or current.start_time != item.start_time:
        return
    try:
        os.kill(item.pid, signum)
    except ProcessLookupError:
        pass


def cleanup(run_dir, grace=5.):
    """逐级停止本次运行的所有进程，重扫可找到 roslaunch 启动的独立会话。"""
    seen = set()
    for signum, duration in ((signal.SIGINT, grace), (signal.SIGTERM, 2.), (signal.SIGKILL, 1.)):
        deadline = time.monotonic() + duration
        sent = set()
        while True:
            items = matching_processes(run_dir)
            if not items:
                return True
            for item in items:
                identity = (item.pid, item.start_time)
                if identity not in seen:
                    print('清理本次进程 pid={} {}'.format(item.pid, item.command[:160]), flush=True)
                    seen.add(identity)
                if identity not in sent:
                    signal_process(item, signum)
                    sent.add(identity)
            if time.monotonic() >= deadline:
                break
            time.sleep(.1)
    remaining = matching_processes(run_dir)
    if remaining:
        print('仍未退出的进程: ' + ', '.join(str(p.pid) for p in remaining), file=sys.stderr)
    return not remaining


def resolve_run(log_root, name):
    """只允许本工作空间日志根目录下的运行目录，避免扩大清理范围。"""
    root = log_root.resolve()
    if name:
        run = Path(name).expanduser().resolve()
    else:
        runs = [p for p in root.glob('aruco_docking_*') if p.is_dir()]
        if not runs:
            raise ValueError('没有找到动态 ArUco 运行目录')
        run = max(runs, key=lambda p: p.stat().st_mtime).resolve()
    if run.parent != root or not run.name.startswith('aruco_docking_') or not run.is_dir():
        raise ValueError('只允许清理本工作空间 logs/aruco_docking_* 目录对应的运行')
    return run


def main():
    """--list 只查看，默认执行清理；不依赖 ROS Master 是否仍在运行。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', nargs='?')
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()
    try:
        root = Path(__file__).resolve().parents[2] / 'logs'
        run = resolve_run(root, args.run_dir)
        print('目标运行目录:', run, flush=True)
        if args.list:
            for item in matching_processes(run):
                print(item.pid, item.command)
            return 0
        return 0 if cleanup(run) else 1
    except (OSError, ValueError) as exc:
        print('清理失败:', exc, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())

#include "ros/ros.h"
#include "asrobot/pwm.h"
#include "asrobot/status.h"
#include "std_msgs/Float64.h"

#include <stdio.h>
#include <iostream>

#define PI     3.1415926

int num;
int pwm_task_flag[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
int count_up_down[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
double count_speed[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
float _run_time[12] = {0};

double old_angle[12]={0.1,0.7,-0.7,0.1,0.7,-0.7,0.1,0.7,-0.7,0.1,0.7,-0.7};
double new_angle[12]={0,0,0,0,0,0,0,0,0,0,0,0};

ros::Publisher servo1_pub;
ros::Publisher servo2_pub;
ros::Publisher servo3_pub;
ros::Publisher servo4_pub;
ros::Publisher servo5_pub;
ros::Publisher servo6_pub;
ros::Publisher servo7_pub;
ros::Publisher servo8_pub;
ros::Publisher servo9_pub;
ros::Publisher servo10_pub;
ros::Publisher servo11_pub;
ros::Publisher servo12_pub;


//asrobot::servo servo;
std_msgs::Float64 channel;
/*******************openservo********************************/
void openservo(uint8_t flag)
{   
//loaded gazebo control
}


/******************openmotor*******************************/
void openmotor(uint8_t flag)
{
//unloaded gazebo control  
}

/*****************control the servos based on ROS publisher-subcriber********************/
void publish(int num)
{


switch (num)
{

case 0:servo1_pub.publish(channel);break;
case 1:servo2_pub.publish(channel);break;
case 2:servo3_pub.publish(channel);break;
case 3:servo4_pub.publish(channel);break;
case 4:servo5_pub.publish(channel);break;
case 5:servo6_pub.publish(channel);break;
case 6:servo7_pub.publish(channel);break;
case 7:servo8_pub.publish(channel);break;
case 8:servo9_pub.publish(channel);break;
case 9:servo10_pub.publish(channel);break;
case 10:servo11_pub.publish(channel);break;
case 11:servo12_pub.publish(channel);break;

}    

}

/************MCU****************/
void pwm_task(int task_num)
{
if(count_up_down[task_num] == 1)
        {

                        if(old_angle[task_num] < new_angle[task_num])
                        {
                                old_angle[task_num] +=  count_speed[task_num];
                                channel.data = old_angle[task_num];
                                publish(task_num);
				//printf("data =%f\n",channel.data);

                        }

                        else
                        {
                                pwm_task_flag[task_num] = 0;
                                count_up_down[task_num] = 0;
                                old_angle[task_num] = new_angle[task_num];
                                channel.data = old_angle[task_num];
                                publish(task_num);
				//printf("data =%f\n",channel.data);
                        }



        }

if(count_up_down[task_num] == 2)
        {

                        if(old_angle[task_num] > new_angle[task_num])
                        {
                                old_angle[task_num] -=  count_speed[task_num];
                                channel.data = old_angle[task_num];
                                publish(task_num);
				//printf("data =%f\n",channel.data);
                        }
                        else
                        {
                                pwm_task_flag[task_num] = 0;
                                count_up_down[task_num] = 0;
                                old_angle[task_num] = new_angle[task_num];
                                channel.data = old_angle[task_num];
                                publish(task_num);
				//printf("data =%f\n",channel.data);
                        }



        }


}



/********************pwmcallback**********************/
void pwmcallback(const asrobot::pwm::ConstPtr& msg)
{
    int length;
    
    length = msg->channel.size();
    
    
    for(int i=0; i<length; i++)
	{       
		num = msg->channel[i];
		new_angle[num] =  msg->duty[i]/180*PI;//get angle and time
                _run_time[num] = msg->time[i];
               if(old_angle[num] != new_angle[num])
		{
			pwm_task_flag[num] = 1;
		}
		else
		{
			pwm_task_flag[num] = 0;
		}

		if(old_angle[num] < new_angle[num])
		{
			count_up_down[num] = 1;
		}
		else if(old_angle[num] > new_angle[num])
		{
			count_up_down[num] = 2;
		}
		else
		{
                        count_up_down[num] = 0;
		}

		if(pwm_task_flag[num] == 1)
		{
                    if(count_up_down[num] == 1)
                    {
                       count_speed[num] = (new_angle[num]-old_angle[num])/_run_time[num];

                    }

                    if(count_up_down[num] == 2)
                    {
                       count_speed[num] = (old_angle[num]-new_angle[num])/_run_time[num];

                    }

                }
		else
		{
			count_speed[num] = 0;
		}
                //std::cout<<"count_speed"<<num<<" = "<<count_speed[num]<<std::endl;
                //std::cout<<"pwm_task_flag"<<i<<" = "<<pwm_task_flag[i]<<std::endl;
	}


	

}
/*********************statuscallback***********************/
void statuscallback(const asrobot::status::ConstPtr& msg)
{
    if(msg->openservo)
    {
        openservo(1);
        printf("open servo!\n");
    }
    else
    {
        openservo(0);
        printf("close servo!\n");
    }

    if(msg->openmotor)
    {
        openmotor(1);
        printf("open motor!\n");
    }
    else
    {
        openmotor(0);
        printf("close motor!\n");
    }
}

/***********************main*****************************/
int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "joint_control");
    ros::NodeHandle n;
    ros::Subscriber pwm_sub = n.subscribe("pwm_message", 1000, pwmcallback);
    ros::Subscriber status_sub = n.subscribe("status_message", 1000, statuscallback);


/********************************************************/
    servo1_pub = n.advertise<std_msgs::Float64>("joint1_position_controller/command", 1000);
    servo2_pub = n.advertise<std_msgs::Float64>("joint2_position_controller/command", 1000);
    servo3_pub = n.advertise<std_msgs::Float64>("joint3_position_controller/command", 1000);
    servo4_pub = n.advertise<std_msgs::Float64>("joint4_position_controller/command", 1000);
    servo5_pub = n.advertise<std_msgs::Float64>("joint5_position_controller/command", 1000);
    servo6_pub = n.advertise<std_msgs::Float64>("joint6_position_controller/command", 1000);
    servo7_pub = n.advertise<std_msgs::Float64>("joint7_position_controller/command", 1000);
    servo8_pub = n.advertise<std_msgs::Float64>("joint8_position_controller/command", 1000);
    servo9_pub = n.advertise<std_msgs::Float64>("joint9_position_controller/command", 1000);
    servo10_pub = n.advertise<std_msgs::Float64>("joint10_position_controller/command", 1000);
    servo11_pub = n.advertise<std_msgs::Float64>("joint11_position_controller/command", 1000);
    servo12_pub = n.advertise<std_msgs::Float64>("joint12_position_controller/command", 1000);




    ros::Rate loop_rate(1000);


    while(ros::ok())
    {
        for(int ii=0; ii<12 ; ii++)
	{
                if(pwm_task_flag[ii] == 1)
		{
                        pwm_task(ii);
		}
 	}       

        ros::spinOnce();
        loop_rate.sleep();

    }


    return 0;
}




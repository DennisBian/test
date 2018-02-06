#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <linux/input.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <iostream>

using namespace std;
#define ABS(x)                         (((x) > 0)?(x):(-(x)))
#define PI                             3.1415

serial::Serial ser;
uint8_t buffer[16];
typedef struct pose{
	float angle;
	float sum;
	float init;
}pose;
pose roll,pitch,yaw;

float num=0;
int change(uint8_t g,uint8_t l)
{ uint8_t cc[2];
cc[0]=l;
cc[1]=g;

return (*(int16_t*)&cc);

}
int serialInit()
{
   try{
     ser.setPort("/dev/ttyUSB1");
     ser.setBaudrate(115200);
     serial::Timeout to = serial::Timeout::simpleTimeout(10);
     ser.setTimeout(to);
     ser.open();
   }catch(serial::IOException& e){
     ROS_ERROR_STREAM("Unable to open port");
     return -1;
   }
   if(ser.isOpen()){
     ROS_INFO_STREAM("Serial Port for IMU initialized");
     return 1;
   }else{
     return -1;
   }
}
int get_pose(uint8_t* buffer)
{
     size_t len = ser.read(buffer,16);
      if(buffer[0]==0xAA && buffer[1]==0xAA&& buffer[2]==0X01 && buffer[3]==0x0C && len == 16)
        {      

		roll.angle=change(buffer[4],buffer[5])/10;
   
               ;

                roll.angle =   roll.angle/180*PI;
 		
cout<< "roll.angle: "<<roll.angle<<"rad"<<endl;
		pitch.angle=change(buffer[6],buffer[7])/10;
		pitch.angle =  pitch.angle/180*PI;
cout<< "pitch.angle: "<<pitch.angle<<"rad"<<endl;
		yaw.angle =change(buffer[8],buffer[9])/10;
		yaw.angle =  yaw.angle/180*PI;
cout<< "yaw.angle: "<<yaw.angle<<"rad"<<endl;
          return 1;                 
       }else{      
cout<< "=============================================" << endl;
return 0;}
  
}
/*
	while(num<=10)
        	{
          	if(get_pose(buffer))
          	 {
		roll.sum += roll.angle;
		pitch.sum += pitch.angle;
             	yaw.sum += yaw.angle;
            	 num++;
         	  }
       	 	}
*/
int main (int argc, char** argv){

        ros::init(argc, argv, "pose");
        ros::NodeHandle nh;

        ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_topic", 1000);

        ros::Time current_time, last_time;
        current_time = ros::Time::now();
        last_time = ros::Time::now();
        sensor_msgs::Imu imu;
	if(!serialInit()){return 0;}
        ros::Rate loop_rate(200);

        while(ros::ok()){
		current_time = ros::Time::now();
        	/*while(num<=10)
        	{
          	if(get_pose(buffer))
          	 {
		roll.sum += roll.angle;
		pitch.sum += pitch.angle;
             	yaw.sum += yaw.angle;
            	 num++;
         	  }
       	 	}
       		 roll.init = roll.sum/num;
       		 pitch.init =pitch.sum/num;
       		 yaw.init = yaw.sum/num;*/
		if(get_pose(buffer))
       		 {
        	roll.angle -= roll.init;
		pitch.angle -= pitch.init;
 		yaw.angle -= yaw.init;
			
          geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromRollPitchYaw( roll.angle,pitch.angle,yaw.angle );
		 imu.orientation.x = goal_quat.x;
                 imu.orientation.y = goal_quat.y;
                 imu.orientation.z = goal_quat.z;
                 imu.orientation.w = goal_quat.w;
          
             cout<<"x,y,z,w:"<<imu.orientation.x<<", " <<imu.orientation.y<<" ,"  <<imu.orientation.z<<" ,"  <<imu.orientation.w <<endl;
   		imu.header.stamp = current_time;
		imu_pub.publish(imu);
		
                 //imu.orientation_covariance = 0;

                 /*imu.angular_velocity.x=0;
                 imu.angular_velocity.y=0;              
                 imu.angular_velocity.z=0;
                 //imu.angular_velocity_covariance=0;

                 imu.linear_acceleration.x=0;
                 imu.linear_acceleration.y=0;              
                 imu.linear_acceleration.z=0;
                 //imu.linear_acceleration_covariance=0;

                 imu.header.stamp=current_time;
                 imu.header.frame_id="pose_imu";
                 
                 imu_pub.publish(imu);
                 //printf("ok--------------------------------------------------------------------- \r\n");*/
        }
	
        ros::spinOnce();
        last_time = current_time;
        loop_rate.sleep();
    }
	ser.close();
}







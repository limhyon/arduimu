// IMU Example.

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#define RED_LED_PIN 5
#define BLUE_LED_PIN 6
#define YELLOW_LED_PIN 7

//ros::NodeHandle_<ArduinoHardware, 2, 2, 300, 330> nh;
ros::NodeHandle nh;
geometry_msgs::PoseStamped pose_msg;

ros::Publisher pub_pose("/pose",&pose_msg);

char frameid_pose[]="/pose";

bool blink = false;

void setup()
{
  nh.initNode();
  nh.advertise(pub_pose); 
  
  pose_msg.header.frame_id = frameid_pose;
  
  pinMode(RED_LED_PIN,OUTPUT); //Red LED
  pinMode(BLUE_LED_PIN,OUTPUT); // Blue LED
  pinMode(YELLOW_LED_PIN,OUTPUT); // Yellow LED
}

void loop()
{
  // fill imu value
  //imu_msg.header.stamp = nh.now();
  
  // quaternion
  //imu_msg.orientation.x = 1.0f;
  //imu_msg.orientation.y = 0.0f;
  //imu_msg.orientation.z = 0.0f;
  //imu_msg.orientation.w = 0.0f;
  
  // angular velocity
  //imu_msg.angular_velocity.x = 0.0f;
  //imu_msg.angular_velocity.y = 1.0f;
 // imu_msg.angular_velocity.z = 2.0f;
  
  // linear acceleration
  //imu_msg.linear_acceleration.x = 0.0f;
  //imu_msg.linear_acceleration.y = 1.0f;
  //imu_msg.linear_acceleration.z = 2.0f;
  
  
  pose_msg.header.stamp = nh.now();
  //pub_imu.publish(&imu_msg);
  pub_pose.publish(&pose_msg);
  
  nh.spinOnce();
  
  if(!blink)
  {
    digitalWrite(RED_LED_PIN,HIGH);
    blink = true;
  }
  else
  {
    digitalWrite(RED_LED_PIN,LOW);
    blink = false;
  }
}

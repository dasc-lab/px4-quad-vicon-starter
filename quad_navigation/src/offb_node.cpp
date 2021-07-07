/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <cmath>

const float RATE = 30.0;

const float XMIN = -1.15;
const float XMAX = 1.30;
const float YMIN = -2.6;
const float YMAX = 1.9;
const float ZMIN = 0.0;
const float ZMAX = 3.5;

float set_x, set_y, set_z, set_qx, set_qy, set_qz, set_qw;
bool received_setPoint = false;

geometry_msgs::PoseStamped mocap_pose, vision_pose, mocap;
mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
  current_state = *msg;
}

bool checkBounds(float x, float y, float z)
{
  if (x < XMIN)
  {
    ROS_WARN("REJECTING DESIRED SETPOINT: X < XMIN");
    return false;
  }

  if (x > XMAX)
  {
    ROS_WARN("REJECTING DESIRED SETPOINT: X > XMAX");
    return false;
  }

  if (y < YMIN)
  {
    ROS_WARN("REJECTING DESIRED SETPOINT: Y < YMIN");
    return false;
  }

  if (y > YMAX)
  {
    ROS_WARN("REJECTING DESIRED SETPOINT: Y > YMAX");
    return false;
  }

  if (z < ZMIN)
  {
    ROS_WARN("REJECTING DESIRED SETPOINT: Z < ZMIN");
    return false;
  }

  if (z > ZMAX)
  {
    ROS_WARN("REJECTING DESIRED SETPOINT: Z > ZMAX");
    return false;
  }

  return true;
}

float norm(float x, float y, float z, float w)
{
  return std::sqrt(x * x + y * y + z * z + w * w);
}

void setpointCallback(const geometry_msgs::Pose msg)
{

  if (checkBounds(msg.position.x, msg.position.y, msg.position.z))
  {
    set_x = msg.position.x;
    set_y = msg.position.y;
    set_z = msg.position.z;

    float n = norm(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);

    set_qx = msg.orientation.x / n;
    set_qy = msg.orientation.y / n;
    set_qz = msg.orientation.z / n;
    set_qw = msg.orientation.w / n;
  }
}

void mocapCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  mocap_pose.header.frame_id = msg->header.frame_id;
  mocap_pose.header.stamp = msg->header.stamp;
  mocap_pose.pose = msg->pose;

  // just to initialize the setpoint,
  // in case you switched to offboard without sending a setpoint
  if (!received_setPoint)
  {
    set_x = mocap_pose.pose.position.x;
    set_y = mocap_pose.pose.position.y;
    set_z = mocap_pose.pose.position.z;
    set_qx = mocap_pose.pose.orientation.x;
    set_qy = mocap_pose.pose.orientation.y;
    set_qz = mocap_pose.pose.orientation.z;
    set_qw = mocap_pose.pose.orientation.w;
    received_setPoint = true;
  }
}

int main(int argc, char **argv)
{
  ROS_INFO("Starting Offboard Control Node");
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber sub_waypoint = nh.subscribe("desired_setpoint", 10, setpointCallback);
  ros::Subscriber sub5 = nh.subscribe("mavros/local_position/pose", 10, mocapCallback);

  ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(RATE);

  ROS_INFO("Waiting for FCU connetion ....");
  // wait for FCU connection
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("FCU connection established");

  while (!received_setPoint)
  {
    ROS_INFO("waiting for setpoint");
    ros::spinOnce();
  }

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = set_x;
  pose.pose.position.y = set_y;
  pose.pose.position.z = set_z;

  pose.pose.orientation.w = set_qw;
  pose.pose.orientation.x = set_qx;
  pose.pose.orientation.y = set_qy;
  pose.pose.orientation.z = set_qz;

  //send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    setpoint_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("Sending Position commands");

  while (ros::ok())
  {

    pose.pose.position.x = set_x;
    pose.pose.position.y = set_y;
    pose.pose.position.z = set_z;

    pose.pose.orientation.w = set_qw;
    pose.pose.orientation.x = set_qx;
    pose.pose.orientation.y = set_qy;
    pose.pose.orientation.z = set_qz;

    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    setpoint_pub.publish(pose);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

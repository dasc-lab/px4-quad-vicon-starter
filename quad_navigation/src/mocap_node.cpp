#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>

using namespace std;

geometry_msgs::PoseStamped vision_pose;
geometry_msgs::TransformStamped mocap_pose;

double previousTime = 0.0, currentTime = 0.0;
const float RATE = 30.0;

bool isLive = false;

ros::Publisher pub;

void mocapCallback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
  mocap_pose.header.frame_id = msg->header.frame_id;
  mocap_pose.header.stamp = msg->header.stamp; //ros::Time::now();

  mocap_pose.transform = msg->transform;
  isLive = true;
}

int main(int argc, char **argv)
{
  std::cout << "Starting Mocap Data Transfer Node" << std::endl;
  ros::init(argc, argv, "mocap_node");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<geometry_msgs::TransformStamped>("vicon/quad/quad", 100, mocapCallback);

  pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 20);

  ros::Rate loop_rate(RATE);

  std::cout << "Starting Data Transfer at " << RATE << " Hz" << std::endl;

  ros::spinOnce();
  
  while (ros::ok())
  {

    if (!isLive)
    {
      ROS_WARN("****** !!! MOCAP DATA IS NOT LIVE !!! ******");
    }

    vision_pose.header.frame_id = "world";
    vision_pose.header.stamp = ros::Time::now();

    vision_pose.pose.position.x = mocap_pose.transform.translation.x;
    vision_pose.pose.position.y = mocap_pose.transform.translation.y;
    vision_pose.pose.position.z = mocap_pose.transform.translation.z;

    vision_pose.pose.orientation.w = mocap_pose.transform.rotation.w;
    vision_pose.pose.orientation.x = mocap_pose.transform.rotation.x;
    vision_pose.pose.orientation.y = mocap_pose.transform.rotation.y;
    vision_pose.pose.orientation.z = mocap_pose.transform.rotation.z;

    pub.publish(vision_pose);
    isLive = false;

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}

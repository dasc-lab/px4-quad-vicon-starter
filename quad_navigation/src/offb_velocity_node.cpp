#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <mavros_msgs/State.h>
#include <cmath>

class SafeVelocityController
{

    // constants
    const float alpha = 2.0;  // approach time constant
    const float delta = 0.05; // noise param
    const float PUBRATE = 30.0; // Hz

    // bounds
    const float XMIN = -1.15;
    const float XMAX = 1.30;
    const float YMIN = -2.6;
    const float YMAX = 1.9;
    const float ZMIN = 0.3;
    const float ZMAX = 3.5;

    // current state:
    float x, y, z;
    bool currentPosRecieved = false;
    bool connected = false;

    // target
    float target_vx = 0.0;
    float target_vy = 0.0;
    float target_vz = 0.0;
    float target_om = 0.0; // yaw rate

    // subscribers and publishers
    ros::Publisher velCommandPub;
    ros::Subscriber desVelocitySub;
    ros::Subscriber currentPosSub;
    ros::Subscriber state_sub;

    void state_cb( mavros_msgs::State const &msg)
    {
        connected = msg.connected;
    }

    // setPoint callback
    void desiredVelocityCallback(geometry_msgs::Twist const &msg)
    {
        target_vx = msg.linear.x;
        target_vy = msg.linear.y;
        target_vz = msg.linear.z;
        target_om = msg.angular.z;
    }

    // current position callback
    void currentPosCallback(geometry_msgs::PoseStamped const &msg)
    {
        x = msg.pose.position.x;
        y = msg.pose.position.y;
        z = msg.pose.position.z;
        currentPosRecieved = true;
    }

    float saturate(float v, float vmin, float vmax)
    {

        v = v > vmin ? v : vmin;
        v = v < vmax ? v : vmax;

        return v;
    }

    float filter_x()
    {

        float vxMax = alpha * (XMAX - x - delta);
        float vxMin = -alpha * (x - XMIN - delta);

        return saturate(target_vx, vxMin, vxMax);
    }

    float filter_y()
    {

        float vyMax = alpha * (YMAX - y - delta);
        float vyMin = -alpha * (y - YMIN - delta);

        return saturate(target_vy, vyMin, vyMax);
    }

    float filter_z()
    {

        float vzMax = alpha * (ZMAX - z - delta);
        float vzMin = -alpha * (z - ZMIN - delta);

	ROS_INFO_STREAM(" FILTER: VZ " << target_vz << " VZMIN " << vzMin << " VZMAX " << vzMax);

        return saturate(target_vz, vzMin, vzMax);
    }

    void publishSafeVelocity(const ros::TimerEvent&)
    {

        geometry_msgs::Twist safeVel;

        safeVel.linear.x = filter_x();
        safeVel.linear.y = filter_y();
        safeVel.linear.z = filter_z();
        safeVel.angular.z = target_om;

        velCommandPub.publish(safeVel);
    }

public:
    explicit SafeVelocityController(ros::NodeHandle &nh)
    {

        // subscribers
        desVelocitySub = nh.subscribe("/desiredVelocity", 10, &SafeVelocityController::desiredVelocityCallback, this);
        currentPosSub = nh.subscribe("/mavros/local_position/pose", 10, &SafeVelocityController::currentPosCallback, this);
        state_sub = nh.subscribe("mavros/state", 10, &SafeVelocityController::state_cb, this);

        // publishers
        velCommandPub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);

        // startup checks
        ROS_INFO("Waiting for FCU connetion ....");
        ros::Rate rate(30.0);

        // wait for FCU connection
        while (ros::ok() && !connected)
        {
            ROS_INFO("Waiting for FCU connetion ....");
            ros::spinOnce();
            rate.sleep();
        }

        ROS_INFO("FCU connection established");
        ROS_INFO("Waiting for current position");
        while (ros::ok() && !currentPosRecieved)
        {
            ROS_INFO("Waiting for current position");
            ros::spinOnce();
            rate.sleep();
        }

        ROS_INFO("START UP CHECKS SUCCESSFUL");

        ros::Timer timer = nh.createTimer(ros::Duration(1.0/PUBRATE), &SafeVelocityController::publishSafeVelocity, this);

	ros::spin();


    }
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Safe Velocity Controller");

    ros::init(argc, argv, "offb_velocity_node");
    ros::NodeHandle nh;

    SafeVelocityController controller(nh);

    ros::spin();
    return 0;
}

# Starter Code to get a drone flying autonomously

## Hardware Setup
- Quadrotor running PX4 on a Pixhawk 2 Cube (Black)
- Jetson Nano connected to drone over UART connection 
- Jetson Nano connected to wifi which is also connected to vicon computer
- Vicon Computer running Vicon Bridge 3.1.3 (later versions might work, unclear)

## PX4 setup

Mostly pertaining to mocap setup.

1. Basic quad setup: basically just follow the steps one by one from top to bottom on the left
- airframe setup: coose quadrotor x, either generic or DJI F450 (for the bigger quads in our lab) - choosing a specific frame means that the PID values are likely to be better
- sensors: run the sensor calibrations
- radio: calibrate the radio to make sure everything is moving in the direction as you wiggle the radio
- flight modes: the useful ones are
  - manual - which is hard to fly, but kind of raw
  - stabilised - (my prefered mode) - keeps the quad as level as possible, but will tend to drift in position
  - position mode - requires GPS/vicon data so that the quad knows where it is, but will generally hold its position and altitude
  - offboard mode - which expects the offboard computer to send waypoints or velocity commands. these commands must be arriving at atleast 2Hz, but we normally send them at atleast 20Hz. 
- safety: set appropriate failsafe settings
2. When creating an object in Vicon Tracker, point the front direction of the quad along the vicons x axis, and create the object. this ensures that the yaw angles are calculated correctly.
3. Within PX4, in QGC 
4. Ensure GPS is not required: set `COM_ARM_WO_GPS` set to `allow arming without GPS`
5. Enable Telem2 connection:
- `MAV_1_CONFIG = TELEM 2`  and then reboot
- `MAV_1_MODE = ONBOARD`
- `SER_TEL2_BAUD = 921600`
6. Enable MOCAP fusion
- `EKF2_AID_MASK` check only the fields `vision position fusion` and `vision yaw fusion` instead of `GPS`
- `EKF2_HGT_MODE` set to `vision` instead of `barometer`
- `EKF2_EV_DELAY` set to 50 ms. Technically, should be measured (follow instructions on px4 guide (https://docs.px4.io/master/en/ros/external_position_estimation.html)
- `EKF2_EVA_NOISE` set to the minimum value (2.86 degrees in our case), ie you are saying that the vicon data is absolutely true
- `EKF2_EVP_NOISE` set to the minimum value (0.01 meters in our case), same as above
7. radio control setup
- Set arm switch
- set kill switch
- set flightmode switch
verify that the correct things are highlighted as you toggle the switches on the radio
8. failsafe set setup
- set everything to terminate/land as appropriate

Something that troubled us for a while was that the device id of the drone was set to 7, but mavros was publising to device 1, and so they couldnt talk to each other.


## Jetson Set up 
- Create a catkin workspace: eg.
```
cd 
mkdir catkin_ws
cd catkin_ws
mkdir src
```
- Clone this repo into the workspace
```
git clone <this repo>
```
- Build the workspace
```
catkin build
```
- Everytime you restart the jetson, you need to run
```
sudo chmod 666 /dev/ttyTHS1
```
which gives the terminal permission to access the serial ports of the jetson. 
On the raspi you can add yourself to the appropriate groups, so you have permissions once you restart the drone. You can:
```
sudo adduser $USER dialout
sudo adduser $USER tty
```

## Check setup
1. Launch the initiate command
```
roslaunch quad_navigation initiate.launch
```
parameters you need to setup:

a) Inside `vicon_bridge/launch/vicon.launch`:
- specify the IP address of the machine that is publishing the vicon data
- specify the frame in which the data should be published. We have set it up to `world`
b) Inside `quad_navigation/launch/initiate.launch`
- specify the address at which mavros communicates with the jetson


2. Check whether you are receiving vicon data
```
rostopic echo /vicon/<quad_name>/<quad_name>
```
Here you should check if the values increase as expected while you physically move the quad around

3. Check if the pixhawk is receiving the vicon data
```
rostopic echo /mavros/local_position/pose
```
and check that the vicon position is the same as the mavros estimated position
Dont forget to check that the roll, pitch and yaw angles are also correct

4. Check that manual/stabilised flight modes are working reliably. 

## Running an experiment

1. Launch the initiate command
```
roslaunch quad_navigation initiate.launch
```

2. Launch the offboard node. 

2a) If you wish to publish waypoints (ie x, y, z, and yaw angles)
```
rosrun quad_navigation offb_node
```
and you can publish desired waypoints to the topic `/desired_setpoint`, with messages of type `geometry_msgs::Pose`

2b) If you wish to publish velocities (ie vx, vy, vz and yaw_rates)
```
rosrun quad_navigation offb_velocity_node
```
and you can publish desired speeds to the topic `/desiredVelocity`, with messages of type of `geometry_msgs::Twist`

Note, I have defined the boundary of the net in our lab inside the code, and have a safety fitler onboard, such that based on the current position, the commanded velocity is modulated so the drone doesnt crash into the nets. 
Technically, one should verify the boundary of the nets each time the nets are set up.

3. Set RC to manual/stabilised mode, take off, and switch into offboard mode, when ready
(Technically you dont need to take off, but if the yaw angle is off, it may not take off reliably. 



## Additional Notes:

Connect pixhawk using raspi to QGC: (make sure QGC is running before you start mavros on the px4)

when launch `mavros` on the raspi, you can use the launch command:
```
roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600 gcs_url:=udp://<PORT NUMBER>@<ip address of ground station> tgt_system:=<vehicle id number (e.g. 1, 2 etc)>
```

QGC by default is listening to `<PORT NUMBER> = 14550`

to use multiple vehicles in the latest version of QGC > application settings > Comm Links > Add. This creates a new port that QGC is listening to

For example, we set 
`Name: ` to the name of the vehicle
`Automatically connect on start` : Yes
`High Latency` : No
`Type`: UDP
`Port Number`: choose a number, incrementing from 14550

For example, for 3 vehicles named vehicle1, vehicle2, vehicle3 with tgt_system id 1, 2, 3 that are publishing on ports 14550, 14551 and 14552, respectively, you specify the Port number in QGC as 14550, 14551, 14552, respectively. 




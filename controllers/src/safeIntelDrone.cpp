#include "controllers/safeIntelDrone.h"

void push(double vx, double vy, double vz, double p, double q, double r){
    Twist t;
    t.linear.x = vx;
    t.linear.y = vy;
    t.linear.z = vz;
    t.angular.x = p;
    t.angular.y = q;
    t.angular.z = r;

    velocities.erase(velocities.begin());
    velocities.push_back(t);
}

Twist filter(){
    Twist t;
    t.linear.x = 0;
    t.linear.y = 0;
    t.linear.z = 0;
    t.angular.x = 0;
    t.angular.y = 0;
    t.angular.z = 0;
    for(vector<Twist>::iterator i = velocities.begin(); i != velocities.end(); ++i){
        t.linear.x += i->linear.x;
        t.linear.y += i->linear.y;
        t.linear.z += i->linear.z;
        t.angular.x += i->angular.x;
        t.angular.y += i->angular.y;
        t.angular.z += i->angular.z;
    }
    t.linear.x /= 10;
    t.linear.y /= 10;
    t.linear.z /= 10;
    t.angular.x /= 10;
    t.angular.y /= 10;
    t.angular.z /= 10;
    return t;
}

void dynamicReconfigureCallback(controllers::setSafeIntelDroneConfig &config, uint32_t level){
    controller = config.controller;
}

void viconCallback(const geometry_msgs::TransformStamped& vicon_msg){
    Duration dt = vicon_msg.header.stamp - time_old;
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.seq = vicon_msg.header.seq;
    odometry_msg.header.stamp = vicon_msg.header.stamp;
    odometry_msg.header.frame_id = vicon_msg.header.frame_id;
    odometry_msg.child_frame_id = "inteldrone";

    odometry_msg.pose.pose.position.x = vicon_msg.transform.translation.x;
    odometry_msg.pose.pose.position.y = vicon_msg.transform.translation.y;
    odometry_msg.pose.pose.position.z = vicon_msg.transform.translation.z;

    tf::Quaternion q(vicon_msg.transform.rotation.x, vicon_msg.transform.rotation.y, vicon_msg.transform.rotation.z, vicon_msg.transform.rotation.w);
    tf::Matrix3x3 m(q);
    double x, y, z, roll, pitch;
    m.getEulerZYX(z, y, x);
    roll = x;
    pitch = y;
    yaw = z;
    odometry_msg.pose.pose.orientation = vicon_msg.transform.rotation;

    double vx = (odometry_msg.pose.pose.position.x - position.x) / (dt.toNSec() / pow(10, 9));     //vx here is not m/s?
    double vy = (odometry_msg.pose.pose.position.y - position.y) / (dt.toNSec() / pow(10, 9));
    double vz = (odometry_msg.pose.pose.position.z - position.z) / (dt.toNSec() / pow(10, 9));
    double omega_x = (roll - orientation.x) / (dt.toNSec() / pow(10, 9));
    double omega_y = (pitch - orientation.y) / (dt.toNSec() / pow(10, 9));
    double omega_z = (yaw - orientation.z) / (dt.toNSec() / pow(10, 9));
    push(vx, vy, vz, omega_x, omega_y, omega_z);
    odometry_msg.twist.twist = filter();
    odometry_publisher.publish(odometry_msg);

    time_old = odometry_msg.header.stamp;
    position = odometry_msg.pose.pose.position;
    orientation.x = roll;
    orientation.y = pitch;
    orientation.z = yaw;

    velocity << vx, vy, vz, omega_z;

    marker_visibile = true;

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = vicon_msg.header;
    pose_msg.pose.position.x = vicon_msg.transform.translation.x;
    pose_msg.pose.position.y = vicon_msg.transform.translation.y;
    pose_msg.pose.position.z = vicon_msg.transform.translation.z;
    pose_msg.pose.orientation = vicon_msg.transform.rotation;
    pose_publisher.publish(pose_msg);
}

float bound(float v, float b){
    if(v < -b)
        return -b;
    if(v > b)
        return b;
    return v;
}

void commandCallback(const std_msgs::Int8& command_msg){

    // change the purpose of the callback: to set up the type of controller

    controller = command_msg.data;


    // geometry_msgs::TwistStamped velocity_msg;
	// velocity_msg.header.stamp = ros::Time::now();;
    // switch(command_msg.data){
    // case 1: // hower

    //     break;
    // case 2: // arm
    // 	offb_set_mode.request.custom_mode = "OFFBOARD";
	// 	set_mode_client.call(offb_set_mode);
   	// 	arm_cmd.request.value = true;
	// 	arming_client.call(arm_cmd);
    //     stop = false;
    //     break;
    // case 3: // land
    // 	offb_set_mode.request.custom_mode = "OFFBOARD";
	// 	set_mode_client.call(offb_set_mode);
    //     land_cmd.request.yaw = yaw;
    //     land_cmd.request.latitude = position_d(0);
    //     land_cmd.request.longitude = position_d(1);
    //     land_cmd.request.altitude = 0;
	// 	land_client.call(land_cmd);
    //     stop = true;
    //     break;
    // case 4: // disarm
	// 	offb_set_mode.request.custom_mode = "OFFBOARD";
	// 	set_mode_client.call(offb_set_mode);
   	// 	arm_cmd.request.value = false;
	// 	arming_client.call(arm_cmd);
	// 	stop = true;
    // }
}

void commandPositionCallback(const geometry_msgs::QuaternionStamped& command_msg){
    float x = bound(command_msg.quaternion.x, MAX_X);
    float y = bound(command_msg.quaternion.y, MAX_Y);
    float z = bound(command_msg.quaternion.z, MAX_Z);
    float psi = bound(command_msg.quaternion.w, M_PI);

    position_d << x, y, z, psi;
}

void commandVelocityCallback(const geometry_msgs::Quaternion& command_msg){
    float vx = bound(command_msg.x, MAX_V);
    float vy = bound(command_msg.y, MAX_V);
    float vz = bound(command_msg.z, MAX_V);
    float r = bound(command_msg.w, MAX_V);

    velocity_d << vx, vy, vz, r;
}

void commandAttitudeCallback(const geometry_msgs::Quaternion& command_msg){
    /*float roll = bound(command_msg.x, MAX_RP);
    float pitch = bound(command_msg.y, MAX_RP);
    float yaw = bound(command_msg.z, M_PI);
    float thrust = bound(command_msg.w, 1);*/

    float roll = bound(command_msg.y, MAX_RP);
    float pitch = bound(command_msg.x, MAX_RP);
    float yaw = bound(command_msg.w, M_PI);
    float thrust = bound(command_msg.z, 1);

    attitude_d << roll, pitch, yaw, thrust;
}

void noiseCallback(const nav_msgs::OdometryConstPtr& noise_msg){
    noise = *noise_msg;
}

void batteryCallback(const sensor_msgs::BatteryStateConstPtr& battery_msg){
    if(battery_msg->percentage < 0.1){
        stop = true;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        set_mode_client.call(offb_set_mode);
        land_cmd.request.yaw = yaw;
        land_cmd.request.latitude = position_d(0);
        land_cmd.request.longitude = position_d(1);
        land_cmd.request.altitude = 0;
        land_client.call(land_cmd);
    }
}

// Constructor
SafeIntelDrone::SafeIntelDrone(int argc, char** argv){
    ros::init(argc, argv, "safeIntelDrone");
    ros::NodeHandle node_handle;

    vicon_subscriber = node_handle.subscribe("/vicon/inteldrone/inteldrone", 1, viconCallback);
    command_subscriber = node_handle.subscribe("/IntelDrone/command", 1, commandCallback);
    command_position_subscriber = node_handle.subscribe("/IntelDrone/command_position", 1, commandPositionCallback);
    command_velocity_subscriber = node_handle.subscribe("/IntelDrone/command_velocity", 1, commandVelocityCallback);
    command_attitude_subscriber = node_handle.subscribe("/IntelDrone/command_attitude", 1, commandAttitudeCallback);
    //noise_subscriber = node_handle.subscribe("/IntelDrone/noise", 1, noiseCallback);
    battery_subscriber = node_handle.subscribe("/mavros/battery", 1, batteryCallback);

    position_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);
    velocity_publisher = node_handle.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1);
    attitude_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 1);
    throttle_publisher = node_handle.advertise<std_msgs::Float64>("/mavros/setpoint_attitude/att_throttle", 1);
    odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("/IntelDrone/odometry", 1);
    pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 1);
	
    set_mode_client = node_handle.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    arming_client = node_handle.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    land_client = node_handle.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    position.x = 0;
    position.y = 0;
    position.z = 0;

    position_d << 0, 0, 0, 0;
    velocity_d << 0, 0, 0, 0;
    attitude_d << 0, 0, 0, 0;

    yaw = 0;

    stop = true;

    controller = 0;

    Twist t;
    velocities.clear();
    for(int i = 0; i < 10; ++i)
        velocities.push_back(t);
}

// Destructor
SafeIntelDrone::~SafeIntelDrone(){
    stop = true;

    ros::Rate rate(10);
    rate.sleep();

    ros::shutdown();
    exit(0);
}

void SafeIntelDrone::run(){
    dynamic_reconfigure::Server<controllers::setSafeIntelDroneConfig> server;
    dynamic_reconfigure::Server<controllers::setSafeIntelDroneConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    double dt = (double)1/100;
    ros::Rate rate(100);
    long count = 0;

    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        geometry_msgs::PoseStamped position_msg;
        geometry_msgs::TwistStamped velocity_msg;
        geometry_msgs::PoseStamped attitude_msg;
        std_msgs::Float64 throttle_msg;
        if(controller == 0){
            position_msg.header.stamp = ros::Time::now();
            position_msg.header.seq = count;
            position_msg.header.frame_id = "local_origin";
            if(position.x < MAX_X && position.x > -MAX_X && position.y < MAX_Y && position.y > -MAX_Y && position.z < MAX_Z && position.z > -MAX_Z){

                double x = position_d(0);
                double y = position_d(1);
                double z = position_d(2);
                double psi  = position_d(3);

                position_msg.pose.position.x = x;
                position_msg.pose.position.y = y;
                position_msg.pose.position.z = z;
                tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, psi);
                quaternionTFToMsg(q, position_msg.pose.orientation);
            }
            else{
                position_msg.pose.position.x = 0;
                position_msg.pose.position.y = 0;
                position_msg.pose.position.z = 0;
                tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
                quaternionTFToMsg(q, position_msg.pose.orientation);
            }

            std::cout << "position command to move to: " << position_msg.pose.position.x << ", " << position_msg.pose.position.y  << ", " 
                                                        << position_msg.pose.position.z << std::endl;

            position_publisher.publish(position_msg);
        }
        else if(controller == 1){
            velocity_msg.header.stamp = ros::Time::now();
            velocity_msg.header.seq = count;
            velocity_msg.header.frame_id = "local_origin";
            if(position.x < MAX_X && position.x > -MAX_X && position.y < MAX_Y && position.y > -MAX_Y && position.z < MAX_Z && position.z > -MAX_Z){

                double vx = velocity_d(0);
                double vy = velocity_d(1);
                double vz = velocity_d(2);
                double r  = velocity_d(3);

                velocity_msg.twist.linear.x = vx;
                velocity_msg.twist.linear.y = vy;
                velocity_msg.twist.linear.z = vz;
                velocity_msg.twist.angular.z = r;
            }
            else{
                velocity_msg.twist.linear.x = 0;
                velocity_msg.twist.linear.y = 0;
                velocity_msg.twist.linear.z = 0;
                velocity_msg.twist.angular.z = 0;
            }

            velocity_publisher.publish(velocity_msg);
        }
        else if(controller == 2){
            attitude_msg.header.stamp = ros::Time::now();
            attitude_msg.header.seq = count;
            attitude_msg.header.frame_id = "local_origin";
            if(position.x < MAX_X && position.x > -MAX_X && position.y < MAX_Y && position.y > -MAX_Y && position.z < MAX_Z && position.z > -MAX_Z){

                double roll = attitude_d(0);
                double pitch = attitude_d(1);
                double yaw = attitude_d(2);
                double thrust = attitude_d(3);

                tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
                //quaternionTFToMsg(q, attitude_msg.pose.orientation);

                throttle_msg.data = thrust;
            }
            else{
                tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
                //quaternionTFToMsg(q, attitude_msg.pose.orientation);

                throttle_msg.data = 0.1;
            }

            attitude_publisher.publish(attitude_msg);
            throttle_publisher.publish(throttle_msg);
        }
        ++count;
    }
}

int main(int argc, char** argv){
    cout << "[SafeIntelDrone] SafeIntelDrone is running..." << endl;

    SafeIntelDrone* IntelDrone = new SafeIntelDrone(argc, argv);

    IntelDrone->run();
}

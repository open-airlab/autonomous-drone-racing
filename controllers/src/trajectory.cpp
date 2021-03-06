#include "controllers/trajectory.h"

void Trajectory::dynamicReconfigureCallback(controllers::setTrajectoryConfig &config, uint32_t level){
    trajectory_type = config.trajectory;
    if(level == 0){
        waypoint = 0;
        if(trajectory_type == 6)
            t = M_PI/2;
        else
            if(trajectory_type == 7)
                t = 1;
            else
                t = 0;
    }

    speed = config.speed;

    pose_d << config.x_d, config.y_d, config.z_d, config.yaw_d / 180 * M_PI;
    std::cout << "control type = " << trajectory_type << "; pose d = " << pose_d(0) << ", " << pose_d(1) << ", " << pose_d(2) << ", " << pose_d(3) << std::endl;
}

// Constructor
Trajectory::Trajectory(int argc, char** argv){
    ros::init(argc, argv, "Trajectory");
    ros::NodeHandle node_handle;

    cmdloop_timer_ = node_handle.createTimer(ros::Duration(0.01), &Trajectory::cmdloopCallback, this); // Define timer for constant loop rate

    trajectory_publisher = node_handle.advertise<geometry_msgs::QuaternionStamped>("/uav/trajectory", 1);
    velocity_publisher = node_handle.advertise<geometry_msgs::QuaternionStamped>("/uav/trajectory_velocity", 1);
    
    pos_publisher_ = node_handle.advertise<geometry_msgs::PoseStamped>("/uav/position", 1);
    flat_publisher_ = node_handle.advertise<controllers::FlatTarget>("reference/flatsetpoint", 10);
    
    odom_publisher_ = node_handle.advertise<nav_msgs::Odometry>("/trajectory/desired_odom", 1); // for rotors_simulator

    pose_d << 0, 0, 0, 0;
    trajectory_type = 0;
    speed = 1;

    ros::Rate rate(100); 

    t = 0;

    // cout << "File name: " << argv[1] << endl;

    // string line;
    // ifstream myfile(argv[1]);
    // if(myfile.is_open()){

    //     int points;
    //     while(getline(myfile, line))
    //         ++points;
    //     waypoints = MatrixXd(points, 4);

    //     ifstream myfile(argv[1]);
    //     for(int i = 0; getline(myfile, line); ++i){
    //         string delimiter = "\t";

    //         size_t pos = 0;
    //         for(int j = 0; (pos = line.find(delimiter)) != string::npos; ++j) {
    //             waypoints(i, j) = atof(line.substr(0, pos).c_str());
    //             line.erase(0, pos + delimiter.length());
    //         }
    //         waypoints(i, 3) = atof(line.c_str()) / 180 * M_PI;
    //     }
    //     myfile.close();
    //     cout << "[Trajectory] waypoints:\n" << waypoints << endl;
    // }
    // else
    //     cout << "Unable to open file: " << argv[1] << endl;

    waypoint = 0;
}

// Destructor
Trajectory::~Trajectory(){
    ros::shutdown();
    exit(0);
}

void Trajectory::cmdloopCallback(const ros::TimerEvent& event){

    geometry_msgs:PoseStamped pose_msg; // for rotors_simulator

    double scale = 2;

    Vector4d trajectory;
    Vector4d velocity;
    
    double R = sqrt(2) / 2;
    Vector4d w1;
    Vector4d w2;

    double var_speed = 0;
    double d = 0;

    // double dt = (double)1/100;
    time_now_ = ros::Time::now();
    double dt = (time_now_ - time_previous_).toSec();
    time_previous_ = time_now_;

    t += speed / scale * dt;

    Matrix3d R1;
    Matrix3d R2;
    switch(trajectory_type){
    case 0: // no command
        trajectory << 0, 0, 0, 0;
        velocity << 0, 0, 0, 0;

        break;
    case 1: // hower
        trajectory << pose_d(0), pose_d(1),  pose_d(2),  pose_d(3);
        velocity << 0, 0, 0, 0;
        break;
    case 2: // user
        trajectory = pose_d;
        velocity << 0, 0, 0, 0;
        break;
    case 3: // waypoints
        if(waypoint < waypoints.rows() - 1){               
            cout << "[Trajectory]: waypoint = " << waypoint << endl;
            //cout << "[Trajectory]: waypoints = " << waypoints << endl;
            //cout << "[Trajectory]: waypoints.row(waypoint) = " << waypoints.row(waypoint).transpose() << endl;
            w1 << waypoints.row(waypoint).transpose();
            //cout << "[Trajectory]: OK1" << endl;
            w2 << waypoints.row(waypoint + 1).transpose();
            //cout << "[Trajectory]: OK2" << endl;
            double d = distance(w1, w2);
            //cout << "[Trajectory]: OK3" << endl;
            trajectory << (1 - t / d) * w1 + t / d * w2;
            //cout << "[Trajectory]: OK4" << endl;
            if(t >= d){
                t = 0;
                waypoint++;
            }
            //cout << "[Trajectory]: OK5" << endl;
        }
        else
            trajectory << waypoints.bottomRows(1).transpose();
        velocity << 0, 0, 0, 0;
        break;
    case 4: // smooth waypoints
        if(waypoint < waypoints.rows() - 1){
            w1 << waypoints.row(waypoint).transpose();
            w2 << waypoints.row(waypoint + 1).transpose();
            //cout << "[Trajectory]: waypoint = " << (waypoint + 1) << " (" << w2.transpose() << ")" << endl;
            double d = distance(w1, w2);
            trajectory << (1 - t / d) * w1 + t / d * w2;
            if(t >= d){
                t = 0;
                waypoint++;
            }
            Vector4d diff = (w2 - w1) / d;
            velocity << speed * diff;
        }
        else{
            trajectory << waypoints.bottomRows(1).transpose();
            velocity << 0, 0, 0, 0;
        }
        break;
    case 5: // circle
        trajectory << scale * sin(t) + pose_d(0), scale * cos(t) + pose_d(1), pose_d(2), pose_d(3);
        velocity << speed * cos(t), -speed * sin(t), 0, 0;
        break;
    case 6: // smooth 8
        trajectory << sqrt(2) / 2 * 4 / (3 - cos(2 * t)) * cos(t) + pose_d(0), sqrt(2) / 2 * 4 / (3 - cos(2 * t)) * cos(t) + pose_d(1), 4 / (3 - cos(2 * t)) * sin(2 * t) / 2 + pose_d(2) + 0.5, pose_d(3);
        velocity << sqrt(2) / 2 * speed * ((4*sin(t))/(cos(2*t) - 3) - (8*sin(2*t)*cos(t))/pow(cos(2*t) - 3, 2)), sqrt(2) / 2 * speed * ((4*sin(t))/(cos(2*t) - 3) - (8*sin(2*t)*cos(t))/pow(cos(2*t) - 3, 2)), 0, 0;
        break;
    case 7: // aggressive 8
        t = fmod(t, 4 + 2 * M_PI * R);
        if(t < 2){
            trajectory << (R - R * t), (R - R * t), 0, -3.0/4 * M_PI + 2 * M_PI;
            velocity << -R, -R, 0, 0;
        }else
            if(t < 2 + M_PI * R){
                trajectory << (-R - sin((t - 2) / R) * R), (-cos((t - 2) / R) * R), 0, M_PI/2;
                velocity << -cos((t - 2) / R) * R, sin((t - 2) / R), 0, 0;
            }else
                if(t < 4 + M_PI * R){
                    trajectory << (-R + R * (t - (2 + M_PI * R))), (R - R * (t - (2 + M_PI * R))), 0, -M_PI/4 + 2 * M_PI;
                    velocity << R, -R, 0, 0;
                }else
                    if(t < 4 + 2 * M_PI * R){
                        trajectory << (R + sin((t - (4 + M_PI * R)) / R) * R), (-cos((t - (4 + M_PI * R)) / R) * R), 0, M_PI/2;
                        velocity << cos((M_PI * R - t + 4) / R), -sin((M_PI * R - t + 4) / R), 0, 0;
                    }
        trajectory = scale * trajectory;
        velocity = speed * velocity;
        R1 << cos(-M_PI / 4), -sin(-M_PI / 4), 0, sin(-M_PI / 4), cos(-M_PI / 4), 0, 0, 0, 1;
        R2 << cos(M_PI / 18), 0, sin(M_PI / 18), 0, 1, 0, -sin(M_PI / 18), 0, cos(M_PI / 18);
        trajectory.head(3) = R1 * R2 * trajectory.head(3);
        velocity.head(3) = R1 * R2 * velocity.head(3);
        trajectory(0) += pose_d(0);
        trajectory(1) += pose_d(1);
        trajectory(2) += pose_d(2);
        trajectory(3) = trajectory(3)/scale - M_PI/4;//pose_d(3);
        break;
    case 8: // square
        t = fmod(t, 8);
        if(t < 2){
            trajectory << 1, 1 - t, 0, 0;
            velocity << 0, -1, 0, 0;
        }
        else
            if(t < 4){
                trajectory << 1 - (t - 2), -1, 0, 0;
                velocity << -1, 0, 0, 0;
            }
            else
                if(t < 6){
                    trajectory << -1, -1 + (t - 4), 0, 0;
                    velocity << 0, 1, 0, 0;
                }
                else
                    if(t < 8){
                        trajectory << -1 + (t - 6), 1, 0, 0;
                        velocity << 1, 0, 0, 0;
                    }
        trajectory = scale * trajectory;
        velocity = speed * velocity;
        R1 << cos(M_PI / 18), 0, sin(M_PI / 18), 0, 1, 0, -sin(M_PI / 18), 0, cos(M_PI / 18);
        trajectory.head(3) = R1 * trajectory.head(3);
        velocity.head(3) = R1 * velocity.head(3);
        trajectory(0) += pose_d(0);
        trajectory(1) += pose_d(1);
        trajectory(2) += pose_d(2);
        trajectory(3) = pose_d(3);
        break;
    case 9: // waypoints-square
        t = fmod(t, scale * 4);
        if(t < scale * 1)
            trajectory << 1, -1, 0.2, 0;
        else
            if(t < scale * 2)
                trajectory << -1, -1, -0.2, 0;
            else
                if(t < scale * 3)
                    trajectory << -1, 1, -0.2, 0;
                else
                    if(t < scale * 4)
                        trajectory << 1, 1, 0.2, 0;
        /*t = fmod(t - (1 - 1.0/scale) * speed * dt, 8);
        if(t < 1)
            trajectory << -1, -1, -0.2, 0;
        else
            if(t < 2)
                trajectory << -1, 0, -0.2, 0;
            else
                if(t < 3)
                    trajectory << -1, 1, -0.2, 0;
                else
                    if(t < 4)
                        trajectory << 0, 1, 0, 0;
                    else
                        if(t < 5)
                            trajectory << 1, 1, 0.2, 0;
                        else
                            if(t < 6)
                                trajectory << 1, 0, 0.2, 0;
                            else
                                if(t < 7)
                                    trajectory << 1, -1, 0.2, 0;
                                else
                                    if(t < 8)
                                        trajectory << 0, -1, 0, 0;*/
        trajectory = scale * trajectory;
        velocity << 0, 0, 0, 0;
        //R1 << cos(-M_PI / 18), 0, sin(-M_PI / 18), 0, 1, 0, -sin(-M_PI / 18), 0, cos(-M_PI / 18);
        //trajectory.head(3) = R1 * trajectory.head(3);
        trajectory(0) += pose_d(0);
        trajectory(1) += pose_d(1);
        trajectory(2) += pose_d(2);
        trajectory(3) = pose_d(3);
        break;
    case 10: // straight line
        t = min(t, 1 * 30 / speed);
        trajectory << t * cos(pose_d(3)), t * sin(pose_d(3)), pose_d(2), pose_d(3);
        velocity << speed * cos(pose_d(3)), speed * sin(pose_d(3)), 0, 0;
        break;
    case 11: // circle with variable speed
        /*t -= speed * dt;
        t += dt;
        if(fmod(t, 2 * M_PI) < M_PI / 2)
            var_speed += 1.0 / (M_PI / 2) * dt;
        else if(fmod(t, 2 * M_PI) < M_PI)
            var_speed = 1.0;
        else if(fmod(t, 2 * M_PI) < 3 * M_PI / 2)
            var_speed += 1.0 / (M_PI / 2) * dt;
        else
            var_speed -= 2.0 / (M_PI / 2) * dt;
        d += var_speed * dt;

        trajectory << sin(d), cos(d), pose_d(2), 0;
        velocity << var_speed * cos(d), -var_speed * sin(d), 0, 0;*/

        t -= speed * dt;
        t += dt;
        if(fmod(t, 2 * M_PI) < M_PI / 2)
            var_speed += 1.0 / (M_PI / 2) * dt;
        else if(fmod(t, 2 * M_PI) < M_PI)
            var_speed = 1.0;
        else if(fmod(t, 2 * M_PI) < 3 * M_PI / 2)
            var_speed += 1.0 / (M_PI / 2) * dt;
        else
            var_speed -= 2.0 / (M_PI / 2) * dt;
        d += var_speed * dt;

        trajectory << 2 * sin(d), 2 * cos(d), pose_d(2), pose_d(3);
        velocity << 2 * var_speed * cos(d), -2 * var_speed * sin(d), 0, 0;
        break;
    case 12: // circle with variable yaw
        trajectory << 2 * sin(t), 2 * cos(t), pose_d(2), pose_d(3) - t + M_PI;
        velocity << speed * cos(t), -speed * sin(t), 0, -speed;
        break;
    }

    double v = sqrt(velocity(0) * velocity(0) + velocity(1) * velocity(1));

    geometry_msgs::QuaternionStamped trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_msg.quaternion.x = trajectory(0);
    trajectory_msg.quaternion.y = trajectory(1);
    trajectory_msg.quaternion.z = trajectory(2);
    trajectory_msg.quaternion.w = trajectory(3);
    trajectory_publisher.publish(trajectory_msg);

    geometry_msgs::QuaternionStamped velocity_msg;
    velocity_msg.header.stamp = ros::Time::now();
    velocity_msg.quaternion.x = velocity(0);
    velocity_msg.quaternion.y = velocity(1);
    velocity_msg.quaternion.z = velocity(2);
    velocity_msg.quaternion.w = var_speed;
    velocity_publisher.publish(velocity_msg);

    controllers::FlatTarget flat_target_msg;
    flat_target_msg.type_mask = 4;
    flat_target_msg.position.x = trajectory(0);
    flat_target_msg.position.y = trajectory(1);
    flat_target_msg.position.z = trajectory(2);
    // flat_target_msg.velocity.x = velocity(0);
    // flat_target_msg.velocity.y = velocity(1);
    // flat_target_msg.velocity.z = velocity(2);
    flat_target_msg.acceleration.x = 0;
    flat_target_msg.acceleration.y = 0;
    flat_target_msg.acceleration.z = 0;
    flat_target_msg.jerk.x = 0;
    flat_target_msg.jerk.y = 0;
    flat_target_msg.jerk.z = 0;
    flat_target_msg.snap.x = 0;
    flat_target_msg.snap.y = 0;
    flat_target_msg.snap.z = 0;
    flat_publisher_.publish(flat_target_msg);

    nav_msgs::Odometry desired_odom_msg;
    desired_odom_msg.header.stamp = ros::Time::now();
    desired_odom_msg.header.frame_id = "map";
    desired_odom_msg.pose.pose.position.x = trajectory(0);
    desired_odom_msg.pose.pose.position.y = trajectory(1);
    desired_odom_msg.pose.pose.position.z = trajectory(2);
    desired_odom_msg.twist.twist.linear.x = velocity(0);
    desired_odom_msg.twist.twist.linear.y = velocity(1);
    desired_odom_msg.twist.twist.linear.z = velocity(2);
    odom_publisher_.publish(desired_odom_msg);

    geometry_msgs::PoseStamped pos_msg;
    pos_msg.header.stamp = ros::Time::now();
    pos_msg.pose.position.x = trajectory(0);
    pos_msg.pose.position.y = trajectory(1);
    pos_msg.pose.position.z = trajectory(2);

    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(trajectory(3), Eigen::Vector3d::UnitZ());

    pos_msg.pose.orientation.x= q.x();
    pos_msg.pose.orientation.y= q.y();
    pos_msg.pose.orientation.z= q.z();
    pos_msg.pose.orientation.w = q.w();
    pos_publisher_.publish(pos_msg);

        // rate.sleep();

        //cout << "[Trajectory]: trajectory = " << trajectory.transpose() << endl;

        //results << ros::Time::now() << ", " << position(0) << ", " << position(1) << ", " << position(2) << ", " << position_real(0) << ", " << position_real(1) << ", " << position_real(2) << ", " << position_d(0) << ", " << position_d(1) << ", " << position_d(2) << endl;

        //cout << "[Trajectory] v_x = " << velocity_msg.x << ", v_y = " << velocity_msg.y << ", v_z = " << velocity_msg.z << endl;
}

double Trajectory::distance(Vector4d v1, Vector4d v2){
    return sqrt(pow(v1(0) - v2(0), 2) + pow(v1(1) - v2(1), 2) + pow(v1(2) - v2(2), 2));
}

int main(int argc, char** argv){
    cout << "[Trajectory] Trajectory generator is running..." << endl;

    Trajectory* trajectory_generator = new Trajectory(argc, argv);

    dynamic_reconfigure::Server<controllers::setTrajectoryConfig> server;
    dynamic_reconfigure::Server<controllers::setTrajectoryConfig>::CallbackType f;
    f = boost::bind(&Trajectory::dynamicReconfigureCallback, trajectory_generator, _1, _2);
    server.setCallback(f);

    // Spin
    ros::spin ();
    return 0;
}

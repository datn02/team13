#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <errno.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include "common.hpp"

double imu_ang_vel = -10, imu_lin_acc = 0; // unlikely to be spinning at -10 at the start
void cbImu(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu_ang_vel = msg->angular_velocity.z;
    imu_lin_acc = msg->linear_acceleration.x;
}

double wheel_l = 10, wheel_r = 10; // init as 10 bcos both are unlikely to be exactly 10 (both can start at non-zero if u reset the sim). whole doubles can also be exactly compared
void cbWheels(const sensor_msgs::JointState::ConstPtr &msg)
{
    wheel_l = msg->position[1]; // double check the topic. it is labelled there
    wheel_r = msg->position[0]; // double check the topic. it is labelled there
}

nav_msgs::Odometry msg_odom;
void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    msg_odom = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_motion");
    ros::NodeHandle nh;

    // Parse ROS parameters
    bool use_internal_odom;
    if (!nh.param("use_internal_odom", use_internal_odom, true))
        ROS_WARN(" TMOVE : Param use_internal_odom not found, set to true");
    bool verbose;
    if (!nh.param("verbose_motion", verbose, false))
        ROS_WARN(" TMOVE : Param verbose_motion not found, set to false");

    // Publisher
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 1, true);

    // Prepare published message
    geometry_msgs::PoseStamped pose_rbt;
    pose_rbt.header.frame_id = "map"; //for rviz

    if (use_internal_odom)
    { // subscribes to odom topic --> is the exact simulated position in gazebo; when used in real life, is derived from wheel encoders (no imu).
        // Subscriber
        ros::Subscriber sub_odom = nh.subscribe("odom", 1, &cbOdom);

        // initialise rate
        ros::Rate rate(25);

        // wait for dependent nodes to load (check topics)
        ROS_INFO("TMOTION: Waiting for topics");
        while (ros::ok() && nh.param("run", true) && msg_odom.header.seq == 0) // dependent on odom
        {
            rate.sleep();
            ros::spinOnce(); //update the topics
        }

        ROS_INFO("TMOTION: ===== BEGIN =====");

        // begin loop
        while (ros::ok() && nh.param("run", true))
        {
            // update topics
            ros::spinOnce();

            // write to published message
            pose_rbt.pose = msg_odom.pose.pose;

            // publish pose
            pub_pose.publish(pose_rbt);

            if (verbose)
            {
                // get ang_rbt from quaternion
                auto &q = pose_rbt.pose.orientation;
                double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
                double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);

                ROS_INFO("TMOTION: Pos(%7.3f, %7.3f)  Ang(%6.3f)  FVel(%6.3f)  AVel(%6.3f)",
                         pose_rbt.pose.position.x, pose_rbt.pose.position.y, atan2(siny_cosp, cosy_cosp),
                         msg_odom.twist.twist.linear.x, msg_odom.twist.twist.angular.z);
            }

            rate.sleep();
        }
    }
    else
    {
        // Parse additional ROS parameters
        Position pos_rbt;
        if (!nh.param("initial_x", pos_rbt.x, 0.0))
            ROS_WARN(" TMOVE : Param initial_x not found, set to 0.0");
        if (!nh.param("initial_y", pos_rbt.y, 0.0))
            ROS_WARN(" TMOVE : Param initial_y not found, set to 0.0");
        double wheel_radius;
        if (!nh.param("wheel_radius", wheel_radius, 0.033))
            ROS_WARN(" TMOVE : Param wheel_radius not found, set to 0.033");
        double axle_track;
        if (!nh.param("axle_track", axle_track, 0.16))
            ROS_WARN(" TMOVE : Param axle_track not found, set to 0.16");
        double weight_odom_v;
        if (!nh.param("weight_odom_v", weight_odom_v, 0.5))
            ROS_WARN(" TMOVE : Param weight_odom_v not found, set to 0.5");
        double weight_odom_w;
        if (!nh.param("weight_odom_w", weight_odom_w, 0.5))
            ROS_WARN(" TMOVE : Param weight_odom_w not found, set to 0.5");
        double weight_imu_v = 1 - weight_odom_v;
        double weight_imu_w = 1 - weight_odom_w;
        double straight_thresh;
        if (!nh.param("straight_thresh", straight_thresh, 0.05))
            ROS_WARN(" TMOVE : Param straight_thresh not found, set to 0.05");
        double motion_iter_rate;
        if (!nh.param("motion_iter_rate", motion_iter_rate, 50.0))
            ROS_WARN(" TMOVE : Param motion_iter_rate not found, set to 50");

        // Subscribers
        ros::Subscriber sub_wheels = nh.subscribe("joint_states", 1, &cbWheels);
        ros::Subscriber sub_imu = nh.subscribe("imu", 1, &cbImu);
        ros::Subscriber sub_odom = nh.subscribe("odom", 1, &cbOdom);

        // initialise rate
        ros::Rate rate(motion_iter_rate); // higher rate for better estimation

        // initialise message for publishing
        pose_rbt.header.frame_id = "world"; // for rviz to visualise model wrt space
        pose_rbt.pose.orientation.x = 0;    // 3 DOF robot; the default is zero anyway, so this line is unnecessary. but kept it here to highlight 3DOF
        pose_rbt.pose.orientation.y = 0;    // 3 DOF robot; the default is zero anyway, so this line is unnecessary. but kept it here to highlight 3DOF

        // wait for dependent nodes to load (check topics)
        ROS_INFO("TMOTION: Waiting for topics");
        while (ros::ok() && nh.param("run", true) && (wheel_l == 10 || wheel_r == 10 || imu_ang_vel == -10)) // dependent on imu and wheels
        {
            rate.sleep();
            ros::spinOnce(); //update the topics
        }

        ROS_INFO("TMOTION: ===== BEGIN =====");

        // declare / initialise other variables
        double ang_rbt = 0; // robot always start at zero.
        double lin_vel = 0, ang_vel = 0;
        double prev_time = ros::Time::now().toSec();
        double dt = 0;
        ////////////////// DECLARE VARIABLES HERE //////////////////
        
        double wheel_l_previous = 0; 
        double wheel_r_previous = 0; // The angle rotated by the wheel(s) in the previous loop
        
        double wheel_l_delta_angle;  // angle displacement of left wheel
        double wheel_r_delta_angle;  // angle displacement of right wheel
        
        double turn_radius; // turn radius of the robot
        
        double odom_velocity; // velocity calculations from odom data
        double odom_angular_rate; // angular speed calculations from odom data
        
        double imu_velocity; // velocity calculations from imu data
        
        double filtered_velocity = 0; // filtered velocity from filter
        double filtered_angular_rate = 0; // filtered angular rate from filter
        
        double ang_rbt_previous = 0; // previous robot angle pose

        // loop
        while (ros::ok() && nh.param("run", true))
        {
            // update topics
            ros::spinOnce();

            dt = ros::Time::now().toSec() - prev_time;
            if (dt == 0) // ros doesn't tick the time fast enough
                continue;
            prev_time += dt;

            ////////////////// MOTION FILTER HERE //////////////////
            
            // Find angular rate and velocity from odom data
            
            wheel_l_delta_angle = wheel_l - wheel_l_previous; // calculating wheel's angle displacement
            wheel_r_delta_angle = wheel_r - wheel_r_previous; // calculating wheel's angle displacement 
            
            
            odom_velocity = (wheel_radius / (2 * dt)) * (wheel_r_delta_angle + wheel_l_delta_angle);   // calculate velocity value from odom data
            odom_angular_rate = (wheel_radius / (axle_track * dt)) * (wheel_r_delta_angle - wheel_l_delta_angle);  // calculate angular rate value from odom data
            
            // Find velocity from imu data
            
            imu_velocity = filtered_velocity + imu_lin_acc * dt;
            
            // weighted average velocity and angular rate filter
            
            filtered_velocity = odom_velocity * weight_odom_v + imu_velocity * weight_imu_v;
            filtered_angular_rate = odom_angular_rate * weight_odom_w + imu_ang_vel * weight_imu_w;
            
            // robot pose calculation
            
            ang_rbt = ang_rbt_previous + filtered_angular_rate * dt;
            
            
            //turn_radius = filtered_velocity / filtered_angular_rate;
            
            if (abs(filtered_angular_rate) > straight_thresh) {  // if robot is spinning
            	turn_radius = filtered_velocity / filtered_angular_rate;
            	pos_rbt.x += turn_radius * (sin(ang_rbt) - sin(ang_rbt_previous));
            	pos_rbt.y += turn_radius * (cos(ang_rbt_previous) - cos(ang_rbt));
            } 
            else {
            	pos_rbt.x += filtered_velocity * dt * cos(ang_rbt_previous);
            	pos_rbt.y += filtered_velocity * dt * sin(ang_rbt_previous);
            }
            
             
            ang_rbt_previous = ang_rbt;
            wheel_l_previous = wheel_l; // setting the current angle to previous
            wheel_r_previous = wheel_r; // setting the current angle to previous
            
             
            // publish the pose
            // inject position and calculate quaternion for pose message, and publish
            pose_rbt.pose.position.x = pos_rbt.x;
            pose_rbt.pose.position.y = pos_rbt.y;
            pose_rbt.pose.orientation.w = cos(ang_rbt / 2);
            pose_rbt.pose.orientation.z = sin(ang_rbt / 2);
            pub_pose.publish(pose_rbt);

            if (verbose)
            {
                auto &q = msg_odom.pose.pose.orientation;
                double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
                double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
                
                ROS_INFO("Pos: True(%7.3f, %7.3f)   Filtered(%7.3f, %7.3f)", msg_odom.pose.pose.position.x, msg_odom.pose.pose.position.y, 
                         pos_rbt.x, pos_rbt.y);
                ROS_INFO("Angle: True(%7.3f)   Filtered(%7.3f)", atan2(siny_cosp, cosy_cosp), ang_rbt);  
                //ROS_INFO("IMU Data: imu_lin_acc = %7.3f  imu_ang_vel = %7.3f", imu_lin_acc, imu_ang_vel);
                //ROS_INFO("Odom Data: odom_velocity = %7.3f  odom_angular_rate = %7.3f", odom_velocity, odom_angular_rate);
            }

            // sleep until the end of the required frequency
            rate.sleep();
        }
    }

    ROS_INFO("TMOTION: ===== END =====");
    return 0;
}

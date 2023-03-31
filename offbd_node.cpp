/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <chrono>
#include <ctime>
#include <cmath>
#include <math.h>
double r=4.0;
double theta;
double count=0.0;
double wn=0.5;
double h;

sensor_msgs::Range current_range;
void ds_cb(const sensor_msgs::Range::ConstPtr& msg){
    current_range = *msg;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //         ("mavros/setpoint_position/local", 10);
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
    // ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
    //         ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Subscriber ds_sub =nh.subscribe<sensor_msgs::Range>("mavros/distance_sensor/lidar",10,ds_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    // 1. target point (3,4,5)

    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 3;
    // pose.pose.position.y = 4;
    // pose.pose.position.z = 5;

    // vel.header.stamp=ros::Time::now();
    // vel.twist.linear.x=0;
    // vel.twist.linear.y=0;
    // vel.twist.linear.z=0;


    //2. circular trajectory
    // mavros_msgs::PositionTarget msg;
    // msg.coordinate_frame=1;
    // msg.header.stamp = ros::Time::now();
    // msg.type_mask=0;
    // msg.position.x=0;
    // msg.position.y=0;
    // msg.position.z=0;
    // msg.velocity.x=0;
    // msg.velocity.y=0;
    // msg.velocity.z=0;
    // msg.acceleration_or_force.x=0;
    // msg.acceleration_or_force.y=0;
    // msg.acceleration_or_force.z=0;

    // x축으로 1씩 전진하게 하는거
    // mavros_msgs::PositionTarget msg;
    // msg.coordinate_frame=1;
    // msg.header.stamp = ros::Time::now();
    // msg.type_mask=0;
    // msg.position.x=0;
    // msg.position.y=0;
    // msg.position.z=1;
    // msg.velocity.x=0;
    // msg.velocity.y=0;
    // msg.velocity.z=0;
    // msg.acceleration_or_force.x=0;
    // msg.acceleration_or_force.y=0;
    // msg.acceleration_or_force.z=0;

    //3. 8-figure trajectory
    mavros_msgs::PositionTarget msg;
    msg.coordinate_frame=1;
    msg.header.stamp = ros::Time::now();
    msg.type_mask=0;
    msg.position.x=0;
    msg.position.y=0;
    msg.position.z=0;
    msg.velocity.x=0;
    msg.velocity.y=0;
    msg.velocity.z=0;
    msg.acceleration_or_force.x=0;
    msg.acceleration_or_force.y=0;
    msg.acceleration_or_force.z=0;

    //4. cylinder trajectory
    // mavros_msgs::PositionTarget msg;
    // msg.coordinate_frame=1;
    // msg.header.stamp = ros::Time::now();
    // msg.type_mask=0;
    // msg.position.x=0;
    // msg.position.y=0;
    // msg.position.z=0;
    // msg.velocity.x=0;
    // msg.velocity.y=0;
    // msg.velocity.z=0;
    // msg.acceleration_or_force.x=0;
    // msg.acceleration_or_force.y=0;
    // msg.acceleration_or_force.z=0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        // local_pos_pub.publish(pose);
        // cmd_vel_pub.publish(vel);
        local_pos_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        // 2. circular trajectory
        // theta=wn*count*0.05;
        // msg.header.stamp=ros::Time::now();
        // msg.coordinate_frame=1;
        // msg.type_mask=0;
        // msg.position.x= r*sin(theta);
        // msg.position.y= r*cos(theta);
        // msg.position.z= 3;
        // msg.velocity.x= 0;
        // msg.velocity.y= 0;
        // msg.velocity.z= 0;
        // msg.acceleration_or_force.x=0;
        // msg.acceleration_or_force.y=0;
        // msg.acceleration_or_force.z=0;

        // x축으로 1씩 전진하게 하는거
        // mavros_msgs::PositionTarget msg;
        // msg.coordinate_frame=1;
        // msg.header.stamp = ros::Time::now();
        // msg.type_mask=0;
        // msg.position.x=count;
        // msg.position.y=0;
        // msg.position.z=1;
        // msg.velocity.x=0;
        // msg.velocity.y=0;
        // msg.velocity.z=0;
        // msg.acceleration_or_force.x=0;
        // msg.acceleration_or_force.y=0;
        // msg.acceleration_or_force.z=0;


        //3. 8-figure trajectory
        theta=wn*count*0.05;
        msg.header.stamp=ros::Time::now();
        msg.coordinate_frame=1;
        msg.type_mask=0;
        msg.position.x= r*sin(theta);
        msg.position.y= r*sin(theta)*cos(theta);
        msg.position.z= 5;
        msg.velocity.x= 0;
        msg.velocity.y= 0;
        msg.velocity.z= 0;
        msg.acceleration_or_force.x=0;
        msg.acceleration_or_force.y=0;
        msg.acceleration_or_force.z=0;

        
        //4. cylinder trajectory
        // theta=wn*count*0.05;
        // msg.coordinate_frame=1;
        // msg.header.stamp = ros::Time::now();
        // msg.type_mask=0;
        // msg.position.x=r*cos(theta);
        // msg.position.y=r*sin(theta);
        // msg.position.z=theta;
        // msg.velocity.x=0;
        // msg.velocity.y=0;
        // msg.velocity.z=0;
        // msg.acceleration_or_force.x=0;
        // msg.acceleration_or_force.y=0;
        // msg.acceleration_or_force.z=0;

        count++;
        //local_pos_pub.publish(pose);
        local_pos_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

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
/*
mavros_msgs는 MAVROS에서 제공하는 토픽이나 메시지들을 포함하고 있음
*/
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
/*
autopilot의 current state를 콜백함 -> arming이나 offboard 모드로 전환할 때 상태 판별하기 위해 사용
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    //state를 계속 subscribe하기 위함

    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //         ("mavros/setpoint_position/local", 10);
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
    //trajectory를 생성하기 위함 (position, velocity, acceleration 값 전달)

    // ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
    //         ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    //arming mode 전환하기 위해 사용
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    // 추후에 offboard 모드로 전환하기 위해 사용

    ros::Subscriber ds_sub =nh.subscribe<sensor_msgs::Range>("mavros/distance_sensor/lidar",10,ds_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    // mavros와 px4_sitl 연결을 위해 기다리는 시간.
    // 연결 되면 위에 선언한 토픽들을 publish하고 subscribe할 것임을 전달하고 해당 while문은 종료됨.
    
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
    //미리 Positiontarget 메시지를 initialize해둠 

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
    //offboard 모드 전환하기 전에 미리 Positiontarget 정보를 보내고 있어야 함
    //그렇지 않으면 전환이 되지 않음

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    //offboard 모드 전환
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    //arming 모드 전환
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){//ros가 돌아가고 있을 때
        if( current_state.mode != "OFFBOARD" && //iris(모델)이 offboard 모드가 아니고
            (ros::Time::now() - last_request > ros::Duration(5.0))){ //ros에서 명령을 전달한 지 5초가 넘었다면
            if( set_mode_client.call(offb_set_mode) && //iris(모델)을 offboard 모드로 전환하라는 명령을 전달하고
                offb_set_mode.response.mode_sent){//offboard 명령이 전달 됐다면
                ROS_INFO("Offboard enabled");//offboard 모드로 전환해라
            }
            last_request = ros::Time::now();
        } else { //offboard로 전환됐고
            if( !current_state.armed && //iris(모델)이 arming 상태가 아니고
                (ros::Time::now() - last_request > ros::Duration(5.0))){// ros에서 명령을 전달한지 5초가 넘었다면
                if( arming_client.call(arm_cmd) && //iris(모델)을 arming시키고
                    arm_cmd.response.success){ //arming 명령이 전달됐다면
                    ROS_INFO("Vehicle armed");//arming 시켜라
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
        ros::spinOnce(); //노드가 꺼지기 전까지 ROS에게 대기 요청하고 콜백을 처리하고 넘어감
                        //쉽게 말하면 ctrl+c 이전까지 콜백 함수 호출하는 것
                        // 여기서 말하는 콜백 함수는 state_cb, 드론의 상태를 계속 받아옴
        rate.sleep(); //while문에서 설정한 주기를 맞추기 위해 기다리는 함수
                        //위의 ros::Rate rate(20.0);가 그 주기
                        //콜백 함수에 들어갔다가 다시 나와서 메인 루프가 20hz로 동작하게끔 하는 것
    }

    return 0;
}

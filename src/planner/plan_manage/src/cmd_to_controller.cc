/**
 * @file cmd_to_controller.cc
 * @brief subscribe /planning/pos_cmd (quadrotor_msgs) topic, share info with outside controller
 * Created by hazy parker on 23-7-18.
 */

#include <ros/ros.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

bool if_cmd_received = false;
quadrotor_msgs::PositionCommand quad_cmd;
void quadrotor_pos_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg){
    quad_cmd = *msg;
    if_cmd_received = true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "cmd_to_controller");
    ros::NodeHandle nodeHandle;

    ros::Subscriber state_sub = nodeHandle.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nodeHandle.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nodeHandle.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Subscriber quad_cmd_sub = nodeHandle.subscribe<quadrotor_msgs::PositionCommand>
            ("/planning/pos_cmd", 100, quadrotor_pos_cb);
    ros::Publisher set_pos_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);

    ros::Rate rate(20);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.8;

    //send a few set points before starting
    for(int i = 200; ros::ok() && i > 0; --i){
        set_pos_pub.publish(pose);
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

        if(if_cmd_received){
            pose.pose.position.x = quad_cmd.position.x;
            pose.pose.position.y = quad_cmd.position.y;
            pose.pose.position.z = quad_cmd.position.z;
        }

        set_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }


}


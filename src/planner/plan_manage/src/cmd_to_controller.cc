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
#include <tf2/LinearMath/Quaternion.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

bool if_init_pose_set = false;
double init_x, init_y;
double init_z = 1.0; // default takeoff height
geometry_msgs::PoseStamped current_pose;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    current_pose = *msg;
    if (!if_init_pose_set){
        ROS_INFO("init pos set to: %f, %f, %f", current_pose.pose.position.x,
                 current_pose.pose.position.y, current_pose.pose.position.z);
        init_x = current_pose.pose.position.x;
        init_y = current_pose.pose.position.y;
        ROS_INFO("takeoff height set to: %f", init_z);
        if_init_pose_set = true;
    }   
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
    ros::Subscriber local_pos_sub = nodeHandle.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, local_pos_cb);

    ros::Rate rate(20);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("fcu connected");

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = init_x;
    pose.pose.position.y = init_y;
    pose.pose.position.z = init_z;

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
    bool if_offboard = false;
    bool if_armed = false;
    bool if_takeoff = false;
    tf2::Quaternion q;

    double origin_x, origin_y, origin_z;
    double origin_yaw;
    nodeHandle.getParam("origin_x", origin_x);
    nodeHandle.getParam("origin_y", origin_y);
    nodeHandle.getParam("origin_z", origin_z);
    nodeHandle.getParam("origin_yaw", origin_yaw);

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" && !if_offboard &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
                if_offboard = true;
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && !if_armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    if_armed = true;
                }
                last_request = ros::Time::now();
            }
        }

        if(if_cmd_received){
            // use ego-planner pos ctrl
            // NOTE: vel need to be lower than 0.5m/s to ensure safety in pos ctrl
            pose.pose.position.x = quad_cmd.position.x;
            pose.pose.position.y = quad_cmd.position.y;
            pose.pose.position.z = quad_cmd.position.z;
            q.setRPY(0, 0, quad_cmd.yaw);
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
        } else{
            if (sqrt(pow(current_pose.pose.position.x - init_x, 2) +
                pow(current_pose.pose.position.x - init_y, 2)+
                pow(current_pose.pose.position.x - init_z, 2)) > 0.2 && !if_takeoff){
                // keep taking off if not in the takeoff area
                pose.pose.position.x = init_x;
                pose.pose.position.y = init_y;
                pose.pose.position.z = init_z;
                // q.setRPY(0, 0, origin_yaw);
                // pose.pose.orientation.x = q.x();
                // pose.pose.orientation.y = q.y();
                // pose.pose.orientation.z = q.z();
                // pose.pose.orientation.w = q.w();
            }else{
                // takeoff done, keep hovering
                pose.pose.position.x = current_pose.pose.position.x;
                pose.pose.position.y = current_pose.pose.position.y;
                pose.pose.position.z = current_pose.pose.position.z;
                if_takeoff = true;
            }

        }

        set_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }
}


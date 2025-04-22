/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/
 
#include "xarm_planner/xarm_planner.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"


void exit_sig_handler(int signum)
{
    fprintf(stderr, "[test_xarm_planner_api_pose] Ctrl-C caught, exit process...\n");
    exit(-1);
}

float get_euclid(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b) {
    std::cout << "euclid a: " << a.position.x << ", " << a.position.y << ", " << a.position.z << std::endl;
    std::cout << "euclid b: " << b.position.x << ", " << b.position.y << ", " << b.position.z <<  std::endl;
    float value = std::pow(std::pow(a.position.x - b.position.x, 2) + std::pow(a.position.y - b.position.y, 2) + std::pow(a.position.z - b.position.z, 2), 0.5);
    std::cout << "euclid val " << value << std::endl;
    return std::pow(std::pow(a.position.x - b.position.x, 2) + std::pow(a.position.y - b.position.y, 2) + std::pow(a.position.z - b.position.z, 2), 0.5);
}

/*
msg = PoseArray()

                    spine_base = Pose()
                    print(frame[8]) # type is numpy.float32
                    spine_base.position.x, spine_base.position.y, spine_base.position.z = frame[8].astype(float)

                    thorax = Pose()
                    print(frame[9]) # type is numpy.float32
                    thorax.position.x, thorax.position.y, thorax.position.z = frame[8].astype(float)

                    neck = Pose()
                    print(frame[10]) # type is numpy.float32
                    neck.position.x, neck.position.y, neck.position.z = frame[8].astype(float)

                    head = Pose()
                    print(frame[11]) # type is numpy.float32
                    head.position.x, head.position.y, head.position.z = frame[8].astype(float)
                    
                    l_shoulder = Pose()
                    print(frame[12]) # type is numpy.float32
                    l_shoulder.position.x, l_shoulder.position.y, l_shoulder.position.z = frame[8].astype(float)

                    l_elbow = Pose()
                    print(frame[13]) # type is numpy.float32
                    l_elbow.position.x, l_elbow.position.y, l_elbow.position.z = frame[8].astype(float)

                    l_wrist = Pose()
                    print(frame[14]) # type is numpy.float32
                    l_wrist.position.x, l_wrist.position.y, l_wrist.position.z = frame[8].astype(float)

                    r_shoulder = Pose()
                    print(frame[17]) # type is numpy.float32
                    r_shoulder.position.x, r_shoulder.position.y, r_shoulder.position.z = frame[8].astype(float)

                    r_elbow = Pose()
                    print(frame[18]) # type is numpy.float32
                    r_elbow.position.x, r_elbow.position.y, r_elbow.position.z = frame[8].astype(float)

                    r_wrist = Pose()
                    print(frame[19]) # type is numpy.float32
                    r_wrist.position.x, r_wrist.position.y, r_wrist.position.z = frame[8].astype(float)
                    
                    msg.poses = [spine_base, thorax, neck, head, l_shoulder, l_elbow, l_wrist, r_shoulder, r_elbow, r_wrist]
                    # self.get_logger().info('Publishing: "%s"' % msg.poses[0].x)
                    self.publisher_.publish(msg)
                    
*/
geometry_msgs::msg::PoseArray joint_positions;
bool received_pose = false;
// Callback function
void joints_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    joint_positions = *msg;
    received_pose = true;
    std::cout << "Received target joints!" << std::endl;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_xarm_planner_api_pose", node_options);
    RCLCPP_INFO(node->get_logger(), "test_xarm_planner_api_pose start");
    
    
    auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseArray>("/joint_positions", 10, joints_callback);


    signal(SIGINT, exit_sig_handler);

    int dof;
    node->get_parameter_or("dof", dof, 7);
    std::string robot_type;
    node->get_parameter_or("robot_type", robot_type, std::string("xarm"));
    std::string group_name = robot_type;
    if (robot_type == "xarm" || robot_type == "lite")
        group_name = robot_type + std::to_string(dof);
    std::string prefix;
    node->get_parameter_or("prefix", prefix, std::string(""));
    if (prefix != "") {
        group_name = prefix + group_name;
    }

    RCLCPP_INFO(node->get_logger(), "namespace: %s, group_name: %s", node->get_namespace(), group_name.c_str());

    xarm_planner::XArmPlanner planner(node, group_name);

    // TODO: SUBSCRIBE TO A POSE MESSAGE !!!!
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.position.x = 0.3;
    target_pose1.position.y = -0.1;
    target_pose1.position.z = 0.2;
    target_pose1.orientation.x = 1;
    target_pose1.orientation.y = 0;
    target_pose1.orientation.z = 0;
    target_pose1.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose2;
    target_pose2.position.x = 0.3;
	target_pose2.position.y = 0.1;
	target_pose2.position.z = 0.2;
	target_pose2.orientation.x = 1;
	target_pose2.orientation.y = 0;
	target_pose2.orientation.z = 0;
	target_pose2.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose3;
    target_pose3.position.x = -0.3;
	target_pose3.position.y = 0.1;
	target_pose3.position.z = 0.4;
	target_pose3.orientation.x = 1;
	target_pose3.orientation.y = 0;
	target_pose3.orientation.z = 0;
	target_pose3.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose4;
    target_pose4.position.x = -0.3;
	target_pose4.position.y = -0.1;
	target_pose4.position.z = 0.4;
	target_pose4.orientation.x = 1;
	target_pose4.orientation.y = 0;
	target_pose4.orientation.z = 0;
	target_pose4.orientation.w = 0;

    // TODO: SUBSCRIBE TO AN OBSTACLE POSITION AND PASS TO addObstacles()

    rclcpp::Rate rate(1); // 1 Hz
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if (received_pose) {

            geometry_msgs::msg::Pose spine_base = joint_positions.poses[0];
            geometry_msgs::msg::Pose thorax = joint_positions.poses[1];
            geometry_msgs::msg::Pose neck = joint_positions.poses[2];
            geometry_msgs::msg::Pose head = joint_positions.poses[3];
            geometry_msgs::msg::Pose l_shoulder = joint_positions.poses[4];
            geometry_msgs::msg::Pose l_elbow = joint_positions.poses[5];
            geometry_msgs::msg::Pose l_wrist = joint_positions.poses[6];
            geometry_msgs::msg::Pose r_shoulder = joint_positions.poses[7];
            geometry_msgs::msg::Pose r_elbow = joint_positions.poses[8];
            geometry_msgs::msg::Pose r_wrist = joint_positions.poses[9];

            // CREATE VECTOR OF PRIMITIVES, VECTOR OF POSES
            // body cylindedr
            shape_msgs::msg::SolidPrimitive body;
            body.type = body.CYLINDER;
            body.dimensions.resize(2);
            body.dimensions[body.CYLINDER_HEIGHT] = get_euclid(thorax, spine_base);
            body.dimensions[body.CYLINDER_RADIUS] = 0.15;
            geometry_msgs::msg::Pose body_pose;
            body_pose.orientation.w = 1.0;
            body_pose.position.x = (thorax.position.x + spine_base.position.x) / 2;
            body_pose.position.y = (thorax.position.y + spine_base.position.y) / 2;
            body_pose.position.z = (thorax.position.z + spine_base.position.z) / 2;

            shape_msgs::msg::SolidPrimitive left_upper;
            left_upper.type = left_upper.CYLINDER;
            left_upper.dimensions.resize(2);
            left_upper.dimensions[left_upper.CYLINDER_HEIGHT] = get_euclid(l_shoulder, l_elbow);
            left_upper.dimensions[left_upper.CYLINDER_RADIUS] = 0.15;
            geometry_msgs::msg::Pose left_upper_pose;
            left_upper_pose.orientation.w = 1.0;
            left_upper_pose.position.x = (l_elbow.position.x + l_shoulder.position.x) / 2;
            left_upper_pose.position.y = (l_elbow.position.y + l_shoulder.position.y) / 2;
            left_upper_pose.position.z = (l_elbow.position.z + l_shoulder.position.z) / 2;

            shape_msgs::msg::SolidPrimitive left_lower;
            left_lower.type = left_lower.CYLINDER;
            left_lower.dimensions.resize(2);
            left_lower.dimensions[left_lower.CYLINDER_HEIGHT] = get_euclid(l_elbow, l_wrist);
            left_lower.dimensions[left_lower.CYLINDER_RADIUS] = 0.15;
            geometry_msgs::msg::Pose left_lower_pose;
            left_lower_pose.orientation.w = 1.0;
            left_lower_pose.position.x = (l_elbow.position.x + l_wrist.position.x) / 2;
            left_lower_pose.position.y = (l_elbow.position.y + l_wrist.position.y) / 2;
            left_lower_pose.position.z = (l_elbow.position.z + l_wrist.position.z) / 2;

            shape_msgs::msg::SolidPrimitive right_upper;
            right_upper.type = right_upper.CYLINDER;
            right_upper.dimensions.resize(2);
            right_upper.dimensions[right_upper.CYLINDER_HEIGHT] = get_euclid(r_shoulder, r_elbow);
            right_upper.dimensions[right_upper.CYLINDER_RADIUS] = 0.15;
            geometry_msgs::msg::Pose right_upper_pose;
            right_upper_pose.orientation.w = 1.0;
            right_upper_pose.position.x = (r_elbow.position.x + r_shoulder.position.x) / 2;
            right_upper_pose.position.y = (r_elbow.position.y + r_shoulder.position.y) / 2;
            right_upper_pose.position.z = (r_elbow.position.z + r_shoulder.position.z) / 2;

            shape_msgs::msg::SolidPrimitive right_lower;
            right_lower.type = right_lower.CYLINDER;
            right_lower.dimensions.resize(2);
            right_lower.dimensions[right_lower.CYLINDER_HEIGHT] = get_euclid(r_elbow, r_wrist);
            right_lower.dimensions[right_lower.CYLINDER_RADIUS] = 0.15;
            geometry_msgs::msg::Pose right_lower_pose;
            right_lower_pose.orientation.w = 1.0;
            right_lower_pose.position.x = (r_elbow.position.x + r_wrist.position.x) / 2;
            right_lower_pose.position.y = (r_elbow.position.y + r_wrist.position.y) / 2;
            right_lower_pose.position.z = (r_elbow.position.z + r_wrist.position.z) / 2;

            std::vector<shape_msgs::msg::SolidPrimitive> primitives = {body, left_upper, left_lower, right_upper, right_lower};
            std::vector<geometry_msgs::msg::Pose> poses = {body_pose, left_upper_pose, left_lower_pose, right_upper_pose, right_lower_pose};

            planner.addObstacles(primitives, poses); //INSERT HERE
            planner.planPoseTarget(target_pose1);
            planner.executePath();

            planner.planPoseTarget(target_pose2);
            planner.executePath();
            planner.planPoseTarget(target_pose3);
            planner.executePath();
            planner.planPoseTarget(target_pose4);
            planner.executePath();
            planner.removeObstacles();
            received_pose = false;
        }
        rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "test_xarm_planner_api_pose over");
    return 0;
}
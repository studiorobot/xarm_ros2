/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
*
* Software License Agreement (BSD License)
*
* Author: Vinman <vinman.cub@gmail.com>
============================================================================*/

#include "xarm_planner/xarm_planner.h"

namespace xarm_planner
{
const double jump_threshold = 0.0;
const double eef_step = 0.005;
const double max_velocity_scaling_factor = 0.3;  // [move_group_interface] default is 0.1
const double max_acceleration_scaling_factor = 0.1;  // [move_group_interface] default is 0.1

XArmPlanner::XArmPlanner(const rclcpp::Node::SharedPtr& node, const std::string& group_name)
    : node_(node)
{
    init(group_name);
}

XArmPlanner::XArmPlanner(const std::string& group_name)
{
    node_ = rclcpp::Node::make_shared("xarm_planner_move_group_node");
    init(group_name);
}

void XArmPlanner::init(const std::string& group_name) 
{
    is_trajectory_ = false;
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);
    RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(node_->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
    RCLCPP_INFO(node_->get_logger(), "Available Planning Groups:");
    std::copy(move_group_->getJointModelGroupNames().begin(), move_group_->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);

    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
}

bool XArmPlanner::planJointTarget(const std::vector<double>& joint_target)
{
    bool success = move_group_->setJointValueTarget(joint_target);
    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setJointValueTarget: out of bounds");
    success = (move_group_->plan(xarm_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "planJointTarget: plan failed");
    is_trajectory_ = false;
    return success;
}

bool XArmPlanner::planPoseTarget(const geometry_msgs::msg::Pose& pose_target)
{
    bool success = move_group_->setPoseTarget(pose_target);
    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setPoseTarget: out of bounds");
    success = (move_group_->plan(xarm_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "planPoseTarget: plan failed");
    is_trajectory_ = false;
    return success;
}

bool XArmPlanner::planPoseTargets(const std::vector<geometry_msgs::msg::Pose>& pose_target_vector)
{
    bool success = move_group_->setPoseTargets(pose_target_vector);
    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setPoseTargets: out of bounds");
    success = (move_group_->plan(xarm_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "planPoseTargets: plan failed");
    is_trajectory_ = false;
    return success;
}

bool XArmPlanner::planCartesianPath(const std::vector<geometry_msgs::msg::Pose>& pose_target_vector)
{   
    // moveit_msgs::msg::RobotTrajectory trajectory;
    
    double fraction = move_group_->computeCartesianPath(pose_target_vector, eef_step, jump_threshold, trajectory_);
    bool success = true;
    if(fraction < 0.9) {
        RCLCPP_ERROR(node_->get_logger(), "planCartesianPath: plan failed, fraction=%lf", fraction);
        return false;
    }
    is_trajectory_ = true;
    // https://github.com/ros-planning/moveit2/commit/8bfe782d6254997d185644fa3eb358d2b79d69b2
    // (struct Plan) trajectory_ => trajectory
    // xarm_plan_.trajectory_ = trajectory;
    return true;
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

bool XArmPlanner::addObstacles(const std::vector<shape_msgs::msg::SolidPrimitive> &primitives, const std::vector<geometry_msgs::msg::Pose>& poses)
{

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    
    for (int i = 0; i < primitives.size(); i++) {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_->getPlanningFrame();
        collision_object.id = "box" + std::to_string(i);;
        collision_object.primitives.push_back(primitives[i]);
        collision_object.primitive_poses.push_back(poses[i]);
        collision_object.operation = collision_object.ADD;
        std::cout << "primtive dimensions " << primitives[i].dimensions[0] << " " << primitives[i].dimensions[1] << std::endl;
        std::cout << "primtive poses " << poses[i].position.x << " " << poses[i].position.y << " " <<  poses[i].position.z << std::endl;

        RCLCPP_INFO(node_->get_logger(), "Add an object into the world");
        collision_objects.push_back(collision_object);
    }

   
    planning_scene_interface_->applyCollisionObjects(collision_objects);

    // TODO: 
    // go through each joint and add an COLLISION OBJECT based on type of joint? 

    // ideal input data: joints[[xyz],[xyz]] where first joint is arm, second joint is torso

    return true;
}

bool XArmPlanner::removeObstacles()
{
    std::vector<std::string> collision_objects = planning_scene_interface_->getKnownObjectNames();
    planning_scene_interface_->removeCollisionObjects(collision_objects);
    RCLCPP_INFO(node_->get_logger(), "Removed objects from world");
}

bool XArmPlanner::executePath(bool wait)
{
    moveit::core::MoveItErrorCode code;
    if (wait)
        code = is_trajectory_ ? move_group_->execute(trajectory_) : move_group_->execute(xarm_plan_);
    else
        code =  is_trajectory_ ? move_group_->asyncExecute(trajectory_) : move_group_->asyncExecute(xarm_plan_);
    bool success = (code == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "executePath: execute failed, wait=%d, MoveItErrorCode=%d", wait, code.val);
    return success;
}
}
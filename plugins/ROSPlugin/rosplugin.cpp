#include "rosplugin.h"
#include <iostream>

#include <ros/ros.h>
#include "std_msgs/String.h"
//#include <control_msgs/FollowJointTrajectoryAction.h>
//#include <actionlib/client/simple_action_client.h>
//#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <eigen3/Eigen/Geometry>
#include <tf2_msgs/TFMessage.h>
#include <QProcess>
#include <QDebug>

#include <actionlib_msgs/GoalStatusArray.h>

void ROSPlugin::initPlugin(int argc, char** argv)
{
    _argc = argc;
    _argv = argv;

}

bool first = true;
void chatterCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
   // if(first)
   // {
        // Initialize moveit
        static const std::string PLANNING_GROUP = "manipulator"; // Reemplaza "manipulator" con el nombre de tu grupo en MoveIt
        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//        const moveit::core::JointModelGroup* joint_model_group =
//             move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
//        first = false;
   // }

        ROS_INFO("EEY");


//    // Getting basic information
//    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
//    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
//    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
//    std::copy(move_group.getJointModelGroupNames().begin(),
//            move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
//    std::cout << std::endl;
}

void ROSPlugin::precalculate()
{

    ROS_INFO("Initializing ROS node");

    ros::init(_argc, _argv, "ros_node");
    _nh = new ros::NodeHandle;
    sleep(1);


}

void ROSPlugin::calculate()
{
    ROS_INFO("Initializing MoveIT");

  //  ros::Subscriber sub = _nh->subscribe("/move_group/status", 1000, chatterCallback);


    // Initialize moveit
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator"; // Reemplaza "manipulator" con el nombre de tu grupo en MoveIt
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//    const moveit::core::JointModelGroup* joint_model_group =
//         move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);



    // Getting basic information
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(),
              move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    std::cout << std::endl;

    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose(move_group.getEndEffectorLink());
    std::cout << "CURRENT POSITION: " << current_pose.pose.position.x <<", " << current_pose.pose.position.y <<", " << current_pose.pose.position.z << std::endl;
    std::cout << "CURRENT ORIENTATION: " << current_pose.pose.orientation.x <<", " << current_pose.pose.orientation.y <<", " << current_pose.pose.orientation.z << ", " << current_pose.pose.orientation.w << std::endl;
    std::cout << "CURRENT FRAME: " << current_pose.header.frame_id << std::endl;

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.x = -0.493563;
    target_pose1.orientation.y = 0.506569;
    target_pose1.orientation.z = -0.497911;
    target_pose1.orientation.w = 0.501863;
    target_pose1.position.x = -0.106875;
    target_pose1.position.y = -0.104085;
    target_pose1.position.z = 0.596322;
    move_group.setPoseTarget(target_pose1);
   // move_group.setGoalTolerance(0.01);

  //  move_group.move();



    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose1);

    geometry_msgs::Pose target_pose3 = target_pose1;

    target_pose3.position.z += 0.05;
    waypoints.push_back(target_pose3);  // down

    target_pose3.position.y += 0.5;
    waypoints.push_back(target_pose3);  // right

    target_pose3.position.z -= 0.1;
    target_pose3.position.y -= 0.5;
    target_pose3.position.x += 0.5;
    waypoints.push_back(target_pose3);  // up and left

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    std::cout << "FRACTION: " << fraction << std::endl;



    move_group.move();



}


void ROSPlugin::postcalculate()
{
    std::cout << "ROS: postcalculate" << std::endl;
    // Cierra ROS
    // Cierra ROS
       if (_nh)
       {
           delete _nh;
           ros::shutdown();
       }
}

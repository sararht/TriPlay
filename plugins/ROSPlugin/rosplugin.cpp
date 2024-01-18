#include "rosplugin.h"
#include <iostream>

#include <ros/ros.h>
#include "std_msgs/String.h"


#include <eigen3/Eigen/Geometry>
#include <tf2_msgs/TFMessage.h>
//#include <QProcess>
//#include <QDebug>

#include <actionlib_msgs/GoalStatusArray.h>

void ROSPlugin::initPlugin(int argc, char** argv, QVector<QVector3D> pos_sensor, QVector<QVector3D> rpy_sensor)
{
    _argc = argc;
    _argv = argv;

    _pos_sensor = pos_sensor;
    _rpy_sensor = rpy_sensor;

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

    // Initialize moveit
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator"; // Reemplaza "manipulator" con el nombre de tu grupo en MoveIt
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // Settings to ease the planning and execution of the trajectories
    move_group.allowReplanning(true);
    move_group.setPlanningTime(12);
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setGoalTolerance(0.001);

    // Getting basic information
    ROS_INFO_NAMED("ROSPlugin", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("ROSPlugin", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("ROSPlugin", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(),
              move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    std::cout << std::endl;

    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose(move_group.getEndEffectorLink());
    std::cout << "CURRENT POSITION: " << current_pose.pose.position.x <<", " << current_pose.pose.position.y <<", " << current_pose.pose.position.z << std::endl;
    std::cout << "CURRENT ORIENTATION: " << current_pose.pose.orientation.x <<", " << current_pose.pose.orientation.y <<", " << current_pose.pose.orientation.z << ", " << current_pose.pose.orientation.w << std::endl;
    std::cout << "CURRENT FRAME: " << current_pose.header.frame_id << std::endl;


    //Initializing RVIZ
    ROS_INFO_NAMED("ROSPlugin", "Visualizing plan 1 as trajectory line");
    moveit_visual_tools::MoveItVisualTools visual_tools(PLANNING_GROUP);
    visual_tools.deleteAllMarkers();

    //Move robot to init pose
    geometry_msgs::Pose init_pose = current_pose.pose;
    init_pose.position.x =-0.1; //-0.1
    init_pose.position.y =0.1; //0.0)
    init_pose.position.z =0.36;  //0.56
    init_pose.orientation.x = 1.0; //0
    init_pose.orientation.y = 0.0; //0
    init_pose.orientation.z = 0.0; //0
    init_pose.orientation.w = 0.0; //1
    move_group.setPoseReferenceFrame("world");

    move_group.setPoseTarget(init_pose);
    move_group.move();
    move_group.stop();
    move_group.clearPoseTarget();
    sleep(1);

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::PoseStamped target_pose1 = move_group.getCurrentPose(move_group.getEndEffectorLink());

    waypoints.push_back(target_pose1.pose);

    int i_ant=0;
    for(int i=1; i<_pos_sensor.size(); i=i+20)
    {
        //Position
//        target_pose1.pose.position.x += (_pos_sensor[i].z() - _pos_sensor[i_ant].z())/1000;
//        target_pose1.pose.position.y -= (_pos_sensor[i].x() - _pos_sensor[i_ant].x())/1000;
//        target_pose1.pose.position.z -= (_pos_sensor[i].y() - _pos_sensor[i_ant].y())/1000;

        target_pose1.pose.position.x = (_pos_sensor[i].z() )/1000;
        target_pose1.pose.position.y = (_pos_sensor[i].x() )/1000;
        target_pose1.pose.position.z = (_pos_sensor[i].y() )/1000;
        double roll = _rpy_sensor[i].z() * M_PI / 180.0;
        double pitch = (_rpy_sensor[i].x()) * M_PI / 180.0;
        double yaw = _rpy_sensor[i].y() * M_PI / 180.0;

        // Configurar la orientación en el mensaje
        tf2::Quaternion quaternion;
        quaternion.setRPY(roll, pitch, yaw);

//        target_pose1.pose.orientation.x = quaternion.getX();
//        target_pose1.pose.orientation.y = quaternion.getY();
//        target_pose1.pose.orientation.z = quaternion.getZ();
//        target_pose1.pose.orientation.w = quaternion.getW();

        target_pose1.header.seq += 1;
        target_pose1.header.stamp = ros::Time::now();


        //Orientation
        std::cout << "middle POSITION: " << target_pose1.pose.position.x <<", " << target_pose1.pose.position.y <<", " << target_pose1.pose.position.z << std::endl;
        std::cout << "middle ORIENTATION: " << target_pose1.pose.orientation.x <<", "
                                            << target_pose1.pose.orientation.y <<", "
                                            << target_pose1.pose.orientation.z <<", "
                                            << target_pose1.pose.orientation.w << std::endl;

        i_ant = i;
        waypoints.push_back(target_pose1.pose);
 //       sleep(0.1);

    }

    move_group.setMaxVelocityScalingFactor(0.01);
    move_group.setMaxAccelerationScalingFactor(0.01);

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    std::cout << "FRACTION: " << fraction << std::endl;

    const moveit::core::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
//    visual_tools.trigger();
//    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


    move_group.execute(trajectory);
 //   std::cout << "N POINTS TRAJ: "<< trajectory.joint_trajectory.points.size() << std::endl;



    current_pose = move_group.getCurrentPose(move_group.getEndEffectorLink());
    std::cout << "FINAL POSITION: " << current_pose.pose.position.x <<", " << current_pose.pose.position.y <<", " << current_pose.pose.position.z << std::endl;
    std::cout << "FINAL ORIENTATION: " << current_pose.pose.orientation.x <<", " << current_pose.pose.orientation.y <<", " << current_pose.pose.orientation.z << ", " << current_pose.pose.orientation.w << std::endl;


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

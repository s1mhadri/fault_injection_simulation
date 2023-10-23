#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ROS
#include <ros/ros.h>

//Time Parametrization
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"

// Random
#include <random>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;


//Functions for Moving and grasping with robot
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Position, Orientation, Planning, Exectuion initPose

void initPose(moveit::planning_interface::MoveGroupInterface& move_group)
{ 
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose_init;

  //Convert Orienation from RPY to Quaternion
  tf2::Quaternion orientation;
  orientation.setRPY( -tau/2, 0, 0);

  target_pose_init.orientation = tf2::toMsg(orientation);
  
  target_pose_init.position.x = 0.5;
  target_pose_init.position.y = 0.0;
  target_pose_init.position.z = 0.5;
  move_group.setPoseTarget(target_pose_init);

  move_group.move();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Position, Orientation, Planning, Exectuion hoverPose

void hoverPose(moveit::planning_interface::MoveGroupInterface& move_group, double x_start, double y_start)
{ 
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose_hover;

  //Convert Orienation from RPY to Quaternion
  tf2::Quaternion orientation;
  orientation.setRPY( -tau/2, 0, 0);

  target_pose_hover.orientation = tf2::toMsg(orientation);
  
  target_pose_hover.position.x = x_start;   // 0.5
  target_pose_hover.position.y = y_start;   // -0.200001
  target_pose_hover.position.z = 0.4;       // 0.4
  move_group.setPoseTarget(target_pose_hover);

  move_group.move();
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Position, Orientation, Planning, Exectuion pickPose as Cartesian Motion

void pickPose(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::string direction){
  moveit::planning_interface::MoveGroupInterface::Plan cartesianPlan;
  move_group_interface.setStartStateToCurrentState();

  move_group_interface.setMaxVelocityScalingFactor(0.2);
  move_group_interface.setMaxAccelerationScalingFactor(0.1);

  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose_pick = move_group_interface.getCurrentPose().pose;
  target_pose_pick.position.x += 0.0;
  target_pose_pick.position.y += 0.0;
  if (direction == "down"){
    target_pose_pick.position.z -= 0.26;//0.26
  }
  else if (direction == "up"){
    target_pose_pick.position.z += 0.26;
  }
  waypoints.push_back(target_pose_pick); 

  moveit_msgs::RobotTrajectory trajectory_msg;
  move_group_interface.setPlanningTime(10.0);
  
 
  double fraction = move_group_interface.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory_msg, false);
  
  // Modify trajectory for adjusting speed
  
  // Create robot trajectory object
  robot_trajectory::RobotTrajectory rt(move_group_interface.getCurrentState()->getRobotModel(), "panda_manipulator");

  // Get robot trajectory
  rt.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory_msg);
 
  // Create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  //Compute TimeStamps
  iptp.computeTimeStamps(rt, 0.1, 0.1);
  
  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory_msg);

  cartesianPlan.trajectory_ = trajectory_msg;
 
  move_group_interface.execute(cartesianPlan);  

  

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Position, Orientation, Planning, Exectuion hoverPlacePose

void hoverPlacePose(moveit::planning_interface::MoveGroupInterface& move_group, double x_final, double y_final)
{ 
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose pose_hover_place;

  //Convert Orienation from RPY to Quaternion
  tf2::Quaternion orientation;
  orientation.setRPY( -tau/2, 0, 0);

  pose_hover_place.orientation = tf2::toMsg(orientation);
  
  pose_hover_place.position.x = x_final;   // 0.5;
  pose_hover_place.position.y = y_final;   // 0.2;
  pose_hover_place.position.z = 0.4;
  move_group.setPoseTarget(pose_hover_place);

  move_group.move();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//Position, Orientation, Planning, Exectuion PlacePose

void PlacePose(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::string direction)
{ 
  moveit::planning_interface::MoveGroupInterface::Plan cartesianPlan;
  move_group_interface.setStartStateToCurrentState();

  move_group_interface.setMaxVelocityScalingFactor(0.2);
  move_group_interface.setMaxAccelerationScalingFactor(0.1);

  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose_place = move_group_interface.getCurrentPose().pose;
  target_pose_place.position.x += 0.0;
  target_pose_place.position.y += 0.0;
  if (direction == "down"){
    target_pose_place.position.z -= 0.26;
  }
  else if (direction == "up"){
    target_pose_place.position.z += 0.26;
  }
  waypoints.push_back(target_pose_place); 

  moveit_msgs::RobotTrajectory trajectory_msg;
  move_group_interface.setPlanningTime(10.0);
 
  double fraction = move_group_interface.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory_msg, false);
  // Modify trajectory for adjusting speed
  
  // Create robot trajectory object
  robot_trajectory::RobotTrajectory rt(move_group_interface.getCurrentState()->getRobotModel(), "panda_manipulator");

  // Get robot trajectory
  rt.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory_msg);
 
  // Create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  //Compute TimeStamps
  iptp.computeTimeStamps(rt, 0.1, 0.1);
  
  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory_msg);

  cartesianPlan.trajectory_ = trajectory_msg;
 
 
  move_group_interface.execute(cartesianPlan);  

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// close Gripper for moveit msg grasp

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{

  // Add both finger joints
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  //set closed position
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.0065;
  posture.points[0].positions[1] = 0.0065;
  posture.points[0].effort.resize(2);
  posture.points[0].effort[0] = 5.00;
  posture.points[0].effort[1] = 5.00;
  
  //Additonal try if force is possible
  //posture.points[0].effort.resize(1);
  //posture.points[0].effort[0] = 1;
  
  
  posture.points[0].time_from_start = ros::Duration(0.5);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//Little movement for grasping with grasp msg

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  //Create Vector for grasp approaches (only need 1)
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);
  
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";

  // Setting grasp pose
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  grasps[0].grasp_pose = current_pose;
  grasps[0].grasp_pose.pose.position.z -= 0.001;
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
 
  //std::cout << "------------Pick Pose----------" << std::endl;
  //std::cout << current_pose << std::endl;

  // Setting pre-grasp approach

  // Direction is set as negative z axis, approach from above the object
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.001;
  grasps[0].pre_grasp_approach.desired_distance = 0.002;

  //Close gripper to grasp object
  closedGripper(grasps[0].grasp_posture);

  // Set support surface as table1.
  move_group.setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  move_group.pick("cylinder1", grasps);
  
}

///////////////////////////////////////////////////////////////////////////////////////
// Plan and execute open hand

void openHand(moveit::planning_interface::MoveGroupInterface& move_group_interface_hand)
{ 
  // Open the gripper
  move_group_interface_hand.setJointValueTarget(move_group_interface_hand.getNamedTargetValues("open"));

  //Move the robot
  // ROS_WARN("START EXECUTION");
  move_group_interface_hand.move();

}

//planning scene
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Objects for planing scene including table, wall back, wall left

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{

  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cylinder will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  // Define the primitive and its dimensions. 
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 1;
  collision_objects[0].primitives[0].dimensions[1] = 1.8;
  collision_objects[0].primitives[0].dimensions[2] = 0;

  // Define the pose of the table. 
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.2;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = -0.01;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;


  collision_objects[0].operation = collision_objects[0].ADD;

  // Add the wall at the back of the robot.
  collision_objects[1].id = "wallback";
  collision_objects[1].header.frame_id = "panda_link0";

  // Define the primitive and its dimensions. 
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0;
  collision_objects[1].primitives[0].dimensions[1] = 1.8;
  collision_objects[1].primitives[0].dimensions[2] = 1.0;

  // Define the pose of the wall. 
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = -0.3;
  collision_objects[1].primitive_poses[0].position.y = 0;
  collision_objects[1].primitive_poses[0].position.z = 0.5;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;


  collision_objects[1].operation = collision_objects[1].ADD;

  // Add the second wall on the left hand side
  collision_objects[2].id = "wallleft";
  collision_objects[2].header.frame_id = "panda_link0";

  // Define the primitive and its dimensions. 
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 1.2;
  collision_objects[2].primitives[0].dimensions[1] = 0;
  collision_objects[2].primitives[0].dimensions[2] = 1.0;

  // Define the pose of the wall on the left. 
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.2;
  collision_objects[2].primitive_poses[0].position.y = -0.9;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;


  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

double getRandomNumber(double min, double max) {
    std::random_device rd;  // Create a random device to seed the generator
    std::mt19937 gen(rd());  // Initialize the Mersenne Twister random number generator

    // Create a uniform distribution for the specified range
    std::uniform_real_distribution<double> distribution(min, max);

    // Generate a random number within the range and return it
    return distribution(gen);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "own_pick_place_V4");
  ros::NodeHandle nh;

  //Get information about robot state
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // ros::Publisher goal_pub = nh.advertise<std_msgs::Bool>("goal_state", 1000);  
  ros::Publisher pose_state_pub = nh.advertise<std_msgs::Int32>("pose_state", 1000); 
  
  ros::WallDuration(1.0).sleep();

  double x_start, y_start, x_final, y_final;
  nh.getParam("x_start", x_start);
  nh.getParam("y_start", y_start);
  nh.getParam("x_final", x_final);
  nh.getParam("y_final", y_final);

  // double min_x = 0.2;
  // double max_x = 0.5;
  // double min_y = 0.1;
  // double max_y = 0.2;

  // double x_final = getRandomNumber(min_x, max_x);
  // double y_final = getRandomNumber(min_y, max_y);
  

  // use for planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  //planning interface
  moveit::planning_interface::MoveGroupInterface group_manipulator("panda_manipulator");
  moveit::planning_interface::MoveGroupInterface group_arm("panda_arm");
  moveit::planning_interface::MoveGroupInterface group_hand("panda_hand");
  
  // Set parameters for group like planner, speed, acceleration
  group_arm.setPlannerId("RRTConnect");
  group_arm.setMaxVelocityScalingFactor(0.2);
  group_arm.setMaxAccelerationScalingFactor(0.1);
  //group_arm.setNumPlanningAttempts(2);

  for (int i = 1; i < 2 ;i = i + 1)
  { 
        
    // Add Objects to the envoirement
    addCollisionObjects(planning_scene_interface);

    //Create Cylinder
    shape_msgs::SolidPrimitive primitive;

    moveit_msgs::CollisionObject object_to_attach;
    object_to_attach.id = "cylinder1";

    shape_msgs::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.145;
    cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.013;
    
    // define the frame/pose for this cylinder
    object_to_attach.header.frame_id = "panda_link0";
    geometry_msgs::Pose grab_pose;
    grab_pose.orientation.w = 1.0;
    grab_pose.position.x = 0.5;
    grab_pose.position.y = -0.2;
    grab_pose.position.z = 0.0725;

    // First, we add the object to the world (without using a vector)
    object_to_attach.primitives.push_back(cylinder_primitive);
    object_to_attach.primitive_poses.push_back(grab_pose);
    object_to_attach.operation = object_to_attach.ADD;
    planning_scene_interface.applyCollisionObject(object_to_attach);

    // Wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Start motion to each position grasp, release and detach

    openHand(group_hand);

    std_msgs::Int32 state;
    state.data = 1;
    pose_state_pub.publish(state);
    hoverPose(group_manipulator, x_start, y_start);

    state.data = 2;
    pose_state_pub.publish(state);
    pickPose(group_manipulator , "down");

    state.data = 3;
    pose_state_pub.publish(state);
    pick(group_arm);

    state.data = 4;
    pose_state_pub.publish(state);
    pickPose(group_manipulator , "up");

    state.data = 5;
    pose_state_pub.publish(state);
    hoverPlacePose(group_manipulator, x_final, y_final);

    state.data = 6;
    pose_state_pub.publish(state);
    PlacePose(group_manipulator , "down");

    state.data = 7;
    pose_state_pub.publish(state);
    openHand(group_hand);
    group_arm.detachObject(object_to_attach.id);

    state.data = 8;
    pose_state_pub.publish(state);
    PlacePose(group_manipulator , "up");

    state.data = 9;
    pose_state_pub.publish(state);
    initPose(group_manipulator);

    state.data = 404;
    pose_state_pub.publish(state);

    ros::WallDuration(2.0).sleep();
    ROS_WARN("round end");

  
  }
  ros::shutdown();
  return 0;
}
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <ros/init.h>
#include <ros/ros.h>

#include <inverse_kinematics_solo12.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/RobotStateJoint.h>
#include <xpp_states/joints.h>
#include <xpp_states/endeffector_mappings.h>

#include <xpp_vis/urdf_visualizer.h>
#include <xpp_vis/cartesian_joint_converter.h>

#include <sensor_msgs/JointState.h>
#include <fstream>
#include <towr_ros/TowrCommand.h>
#include <towr_ros/topic_names.h>

using namespace xpp;
using namespace quad;

int msgCount = 0;
std::fstream fileHandle;
const char *csvFilePath = "/home/lainelab/code/jointStates.csv"; //HARD-CODED PATH

void jointStateCallback(const xpp_msgs::RobotStateJoint &msg);
void UserCmdCallback(const towr_ros::TowrCommand &msg);


int main(int argc, char *argv[])
{
  //Prints the location of the joint state CSV file
  ROS_INFO("JOINTSTATES.CSV IS LOCATED AT %s", csvFilePath);

  ::ros::init(argc, argv, "solo12_urdf_visualizer");

  const std::string solo12_joint_msg_topic = "xpp/joint_solo12_des";

  auto solo12_ik = std::make_shared<InverseKinematicsSolo12>();
  CartesianJointConverter inv_kin_converter(solo12_ik, xpp_msgs::robot_state_desired,
                                            solo12_joint_msg_topic);

  std::vector<UrdfVisualizer::URDFName> joint_names{"FL_HAA", "FL_HFE", "FL_KFE", 
                                                    "FR_HAA", "FR_HFE", "FR_KFE",
                                                    "HL_HAA", "HL_HFE", "HL_KFE",
                                                    "HR_HAA", "HR_HFE", "HR_KFE"};

  std::string urdf_param = "solo12_rviz_urdf_robot_description";
  UrdfVisualizer solo12_des(urdf_param, joint_names, "base_link", "world",
			  solo12_joint_msg_topic, "solo12");

  ::ros::NodeHandle n;
  ::ros::Subscriber userCmdSub = n.subscribe(towr_msgs::user_command, 1, UserCmdCallback);
  ::ros::Subscriber jointStateSub = n.subscribe("xpp/joint_solo12_des", 30, jointStateCallback);

  ::ros::spin();

  return 1;
}


/*
  Writes the joint state to the joint state CSV file.
  Is invoked every time a Solo12 joint state message is received. The joint state messages contain 
  the joint angles needed for visualization, and are published by urdf_visualizer.h every a 
  trajectory with Solo12 as the visualization is run in towr.
*/
void jointStateCallback(const xpp_msgs::RobotStateJoint &msg) 
{
  sensor_msgs::JointState joint_states = msg.joint_state;
  auto positions = joint_states.position;

  fileHandle.open(csvFilePath, std::ios_base::app);


  fileHandle << msg.time_from_start.toSec() << " ";

  for (int i = 0; i < 11; i++){
    fileHandle << positions.at(i) << " ";
  }

  fileHandle << positions.at(11) << "\n";
  fileHandle.close();
}


/*
  Clears the joint state file each time the user presses a key in the towr Xterm interface.
  This means the joint state file is empty if the last key pressed wasn't 'o' (optimize motion) 
  or 'v' (visualize motion).
*/
void UserCmdCallback(const towr_ros::TowrCommand &msg) 
{
  if(msg.optimize || msg.play_initialization){
    ROS_INFO("Clearing jointStates.csv");
    remove(csvFilePath);
  }
}
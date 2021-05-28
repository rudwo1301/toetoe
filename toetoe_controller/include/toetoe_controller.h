#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "mujoco_ros_msgs/JointSet.h"
#include "mujoco_ros_msgs/SensorState.h"
#include <Eigen/Eigen>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "math_type_define.h"
#include <fstream>
#include <iostream>

std::ofstream ankle_roll_trajectory_file;
std::ofstream ankle_roll_real_file;

ros::Publisher mujoco_sim_command_pub_;
ros::Publisher mujoco_joint_set_pub_;
ros::Subscriber mujoco_joint_state_sub_;
ros::Subscriber mujoco_sensor_state_sub_;
ros::Subscriber mujoco_sim_command_sub_;
ros::Subscriber mujoco_sim_time_sub_;

mujoco_ros_msgs::JointSet mujoco_joint_set_msg_;

void parameterSetting();

void desiredAnkleTrajectory();

void desiredPositionTrajectory();

void computeIK(Eigen::Isometry3d com_Position_Trajectory);

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
void sensorStateCallback(const mujoco_ros_msgs::SensorStateConstPtr& msg);
void simCommandCallback(const std_msgs::StringConstPtr& msg);
void simTimeCallback(const std_msgs::Float32ConstPtr& msg);

void jointTorqueSetCommand();
void jointPositionSetCommand();
void simready();
void wait();

bool sim_running;
bool mujoco_ready = false;
bool mujoco_init_receive = false;
bool mujoco_reset = false;
float mujoco_sim_time;
float mujoco_sim_last_time;
float sim_time;

const double hz_ = 200.0;

unsigned int motion_tick_ = 0;
double start_t_ = 0.0;
double start_tick_;

double end_t_;
double end_tick_;

double run_time_;

unsigned int MODEL_DOF_WITH_VIRTUAL_JOINT;
unsigned int MODEL_DOF;
unsigned int VIRTUAL_DOF = 6;

double ankle_pitch_input;
double ankle_roll_input;

Eigen::VectorXd current_q_;
Eigen::VectorXd current_q_dot_;

Eigen::VectorXd current_torque_;

Eigen::VectorXd desired_q_;
Eigen::VectorXd desired_q_dot_;

Eigen::VectorXd desired_torque_;

Eigen::Isometry3d com_Trajectory;

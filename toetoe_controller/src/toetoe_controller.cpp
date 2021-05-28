#include "toetoe_controller.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "toetoe_controller");
    ros::NodeHandle nh;

    RigidBodyDynamics::Model ToeToeModel;
    
    std::string desc_package_path = ros::package::getPath("toetoe_description");
    std::string urdf_path = desc_package_path + "/robots/toetoe.urdf";
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &ToeToeModel,true,false);
    
    ROS_INFO("URDF Loaded from %s", urdf_path.c_str());
    ROS_INFO("Succesfully loaded.");
    MODEL_DOF_WITH_VIRTUAL_JOINT = ToeToeModel.dof_count;
    MODEL_DOF = MODEL_DOF_WITH_VIRTUAL_JOINT - VIRTUAL_DOF;
    ROS_INFO("TOTAL DOF : %d", MODEL_DOF);

    parameterSetting();

    mujoco_joint_state_sub_ = nh.subscribe("/mujoco_ros_interface/joint_states",5,&jointStateCallback, ros::TransportHints().tcpNoDelay(true));
    mujoco_sensor_state_sub_ = nh.subscribe("/mujoco_ros_interface/sensor_states",1,&sensorStateCallback,ros::TransportHints().tcpNoDelay(true));
    mujoco_sim_command_sub_ = nh.subscribe("/mujoco_ros_interface/sim_command_sim2con",5,&simCommandCallback,ros::TransportHints().tcpNoDelay(true));
    mujoco_sim_time_sub_ = nh.subscribe("/mujoco_ros_interface/sim_time",1,&simTimeCallback, ros::TransportHints().tcpNoDelay(true));

    mujoco_sim_command_pub_ = nh.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_con2sim",5);
    mujoco_joint_set_pub_ = nh.advertise<mujoco_ros_msgs::JointSet>("/mujoco_ros_interface/joint_set",5);

    //control mode
    int control_mode = 0;//0 position, 1 torque

    ankle_roll_input = 10*DEG2RAD;
    ankle_pitch_input = 10*DEG2RAD;
    
    desiredSetting();

    ros::Rate loop_rate(200);
    while(ros::ok())
    {
        if(control_mode == 0)
        {
            jointPositionSetCommand();
        }
        else if(control_mode == 1)
        {
            jointTorqueSetCommand();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void parameterSetting()
{
    current_q_.resize(MODEL_DOF);
    current_q_.setZero();

    current_q_dot_.resize(MODEL_DOF);
    current_q_dot_.setZero();

    current_torque_.resize(MODEL_DOF);
    current_torque_.setZero();


    desired_q_.resize(MODEL_DOF);
    desired_q_.setZero();

    desired_q_dot_.resize(MODEL_DOF);
    desired_q_dot_.setZero();

    desired_torque_.resize(MODEL_DOF);
    desired_torque_.setZero();

    mujoco_joint_set_msg_.position.resize(MODEL_DOF);
    mujoco_joint_set_msg_.torque.resize(MODEL_DOF);
}

void desiredSetting()
{
    std::cout << "init : " << mujoco_init_receive << std::endl;
    std::cout << "desired" << std::endl;
    desired_q_(0) = ankle_pitch_input; //ankle_pitch
    std::cout << desired_q_(0) << std::endl<<std::endl;
    desired_q_(1) = ankle_roll_input;//ankle_roll
    ros::Duration(5).sleep();
    std::cout << "init : " << mujoco_init_receive << std::endl;    
}

void sensorStateCallback(const mujoco_ros_msgs::SensorStateConstPtr& msg)
{

}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for(int i = 0; i < MODEL_DOF; i++)
    {
        current_q_(i) = msg->position[i+VIRTUAL_DOF];
        current_q_dot_(i) = msg->velocity[i+VIRTUAL_DOF];
        current_torque_(i) = msg->effort[i+VIRTUAL_DOF];
    }

    if(mujoco_init_receive == false)
    {
        std::cout << "here" << std::endl;
        for(int i=0; i <MODEL_DOF; i++)
        {
            desired_q_(i) = current_q_(i);
        }
        std::cout << desired_q_(0) << std::endl << std::endl;
        mujoco_init_receive = true;
    }
}

void simCommandCallback(const std_msgs::StringConstPtr& msg)
{
    std::string buf;
    buf = msg->data;

    ROS_INFO("CB from simulator : %s", buf.c_str());
    std::cout << "mujoco init receive : " << mujoco_init_receive << std::endl;
    if(buf == "RESET")
    {
        mujoco_sim_last_time = 0.0;
        mujoco_ready = true;

        std_msgs::String rst_msg_;
        rst_msg_.data = "RESET";
        mujoco_sim_command_pub_.publish(rst_msg_);
    }

    if(buf == "INIT")
    {
        mujoco_init_receive = true;
        std_msgs::String rst_msg_;
        rst_msg_.data = "INIT";
        mujoco_sim_command_pub_.publish(rst_msg_);
        mujoco_sim_time = 0.0;
        mujoco_reset = true;
    }
}

void simTimeCallback(const std_msgs::Float32ConstPtr& msg)
{
    mujoco_sim_time = msg->data;
}

void jointTorqueSetCommand()
{

}

void jointPositionSetCommand()
{
    mujoco_joint_set_msg_.MODE = 0;
    if(mujoco_init_receive == true)
    {
        for(int i = 0; i < MODEL_DOF; i++)
        {
            mujoco_joint_set_msg_.position[i] = desired_q_(i);
        }
        //std::cout << "desired ankle roll : "<< desired_q_(1) << std::endl;
        //std::cout << "current ankle roll : "<< current_q_(1) << std::endl;
        //std::cout << "desired ankle pitch : "<< desired_q_(0) << std::endl;
        //std::cout << "current ankle pitch : "<< current_q_(0) << std::endl;

        mujoco_joint_set_msg_.header.stamp = ros::Time::now();
        mujoco_joint_set_msg_.time = mujoco_sim_time;
        mujoco_joint_set_pub_.publish(mujoco_joint_set_msg_);
        mujoco_sim_last_time = mujoco_sim_time;
    }
}
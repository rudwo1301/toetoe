#include "toetoe_controller.h"

int main(int argc, char** argv)
{
    ankle_roll_trajectory_file.open("/home/econom2/Desktop/matlab/data/ankle_roll_tra.txt");
    ankle_roll_real_file.open("/home/econom2/Desktop/matlab/data/ankle_roll_real.txt");

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

    run_time_ = 5.0;
    end_t_ = start_t_ + run_time_;

    ankle_roll_input = 10*DEG2RAD;
    ankle_pitch_input = 10*DEG2RAD;

    parameterSetting();

    mujoco_joint_state_sub_ = nh.subscribe("/mujoco_ros_interface/joint_states",5,&jointStateCallback, ros::TransportHints().tcpNoDelay(true));
    mujoco_sensor_state_sub_ = nh.subscribe("/mujoco_ros_interface/sensor_states",1,&sensorStateCallback,ros::TransportHints().tcpNoDelay(true));
    mujoco_sim_command_sub_ = nh.subscribe("/mujoco_ros_interface/sim_command_sim2con",5,&simCommandCallback,ros::TransportHints().tcpNoDelay(true));
    mujoco_sim_time_sub_ = nh.subscribe("/mujoco_ros_interface/sim_time",1,&simTimeCallback, ros::TransportHints().tcpNoDelay(true));

    mujoco_sim_command_pub_ = nh.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_con2sim",5);
    mujoco_joint_set_pub_ = nh.advertise<mujoco_ros_msgs::JointSet>("/mujoco_ros_interface/joint_set",5);

    mujoco_sim_time = 0.0;
    simready();

    //control mode
    int control_mode = 0;//0 position, 1 torque
    int position_mode = 0;//0 ankle, 1 com position

    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        if(control_mode == 0)
        {
            if(position_mode == 0)
                desiredAnkleTrajectory();
            else if(position_mode == 1)
                desiredPositionTrajectory();
                computeIK(com_Trajectory);
            
            jointPositionSetCommand();
            ankle_roll_real_file << current_q_(0) << std::endl;
        }
        else if(control_mode == 1)
        {
            jointTorqueSetCommand();
        }
        ros::spinOnce();
        //loop_rate.sleep();
        wait();
        motion_tick_++;
    }

    ankle_roll_trajectory_file.close();
    ankle_roll_real_file.close();

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

    com_Trajectory.setIdentity();
    start_tick_ = start_t_ * hz_;
    end_tick_ = end_t_ * hz_;
}

void desiredAnkleTrajectory()
{
    //ankle_pitch
    if(motion_tick_ <(end_tick_-start_tick_)/2)
        desired_q_(0) = DyrosMath::cubic(motion_tick_, start_tick_, (end_tick_ - start_tick_)/2-1, 0.0, ankle_pitch_input, 0.0, 0.0);
    else if(motion_tick_ >=(end_tick_-start_tick_)/2 && motion_tick_ <=end_tick_)
        desired_q_(0) = DyrosMath::cubic(motion_tick_, (end_tick_-start_tick_)/2, end_tick_, ankle_pitch_input, 0.0, 0.0, 0.0);
    //std::cout << desired_q_(0) << std::endl;
    ankle_roll_trajectory_file << desired_q_(0) << std::endl;
    
    //ankle_roll
    desired_q_(1) = 0.0;
    //DyrosMath::cubic(motion_tick_, start_tick_, end_tick_, 0.0, 0.0, ankle_roll_input, 0.0);
}

void desiredPositionTrajectory()
{

}

void computeIK(Eigen::Isometry3d com_Position_Trajectory)
{

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
        for(int i=0; i <MODEL_DOF; i++)
        {
            desired_q_(i) = current_q_(i);
        }
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
        
        ros::Rate poll_rate(100);
        while(!mujoco_init_receive &&ros::ok()){
            ros::spinOnce();
            poll_rate.sleep();
        }
        mujoco_init_receive=false;
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

void simready()
{
    ros::Rate poll_rate(100);
    while(!mujoco_ready && ros::ok())
    {
        ros::spinOnce();
        poll_rate.sleep();
    }
}

void wait()
{
    bool test_b = false;

    ros::Rate poll_rate(20000);
    int n = 0;

    ROS_INFO_COND(test_b, " wait loop enter");
    while ((mujoco_sim_time < (mujoco_sim_last_time + 1.0 / hz_)) && ros::ok())
    {
        ros::spinOnce();
        poll_rate.sleep();
        n++;
    }
    ROS_INFO_COND(test_b, " wait loop exit with n = %d", n);

    while((mujoco_sim_time<(mujoco_sim_last_time+1.0/hz_))&&ros::ok()){
        //ROS_INFO("WAIT WHILE");
        ros::spinOnce();
        poll_rate.sleep();

    }
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
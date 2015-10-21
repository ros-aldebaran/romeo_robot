/**
Copyright (c) 2014, Konstantinos Chatzilygeroudis
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer 
    in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived 
    from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include <iostream>
#include "romeo_dcm_driver/romeo.h"
#include <alerror/alerror.h>
#include <alcommon/albroker.h>
#include <sensor_msgs/JointState.h>
#include <algorithm>

#include <controller_manager_msgs/ControllerState.h>

Romeo::Romeo(boost::shared_ptr<AL::ALBroker> broker, const string &name)
    : AL::ALModule(broker,name),is_connected_(false)
{
    setModuleDescription("Romeo Robot Module");

    functionName("brokerDisconnected", getName(), "Callback when broker disconnects!");
    BIND_METHOD(Romeo::brokerDisconnected);
}

Romeo::~Romeo()
{
    if(is_connected_)
        disconnect();
}

bool Romeo::initialize()
{

    //Romeo Joints Initialization
    const char* joint[] = { "NeckYaw",
                            "NeckPitch",
                            "HeadPitch",
                            "HeadRoll",
                            "LShoulderPitch",
                            "LShoulderYaw",
                            "LElbowRoll",
                            "LElbowYaw",
                            "LWristRoll",
                            "LWristYaw",
                            "LWristPitch",
                            "LHand",
                            "TrunkYaw",
                            "LHipYaw",
                            "LHipRoll",
                            "LHipPitch",
                            "LKneePitch",
                            "LAnklePitch",
                            "LAnkleRoll",
                            "RHipYaw",
                            "RHipRoll",
                            "RHipPitch",
                            "RKneePitch",
                            "RAnklePitch",
                            "RAnkleRoll",
                            "RShoulderPitch",
                            "RShoulderYaw",
                            "RElbowRoll",
                            "RElbowYaw",
                            "RWristRoll",
                            "RWristYaw",
                            "RWristPitch",
                            "RHand" };
    joint_names_ = vector<string>(joint, end(joint));
    
    for(vector<string>::iterator it=joint_names_.begin();it!=joint_names_.end();it++)
    {
        joints_names_.push_back("Device/SubDeviceList/"+(*it)+"/Position/Sensor/Value");
    }
    number_of_joints_ = joint_names_.size();

    // DCM Motion Commands Initialization
    try
    {
        // Create Motion Command
        commands_.arraySetSize(4);
        commands_[0] = string("Joints");
        commands_[1] = string("ClearAll");
        commands_[2] = string("time-mixed");
        commands_[3].arraySetSize(number_of_joints_);

        // Create Joints Actuators Alias
        AL::ALValue commandAlias;
        commandAlias.arraySetSize(2);
        commandAlias[0] = string("Joints");
        commandAlias[1].arraySetSize(number_of_joints_);
        for(int i=0;i<number_of_joints_;i++)
        {
            commandAlias[1][i] = string("Device/SubDeviceList/"+joint_names_[i]+"/Position/Actuator/Value");
            commands_[3][i].arraySetSize(1);
            commands_[3][i][0].arraySetSize(2);
        }
        dcm_proxy_->callVoid("createAlias",commandAlias);

        // Create Joints Hardness Alias
        commandAlias[0] = string("JointsHardness");
        commandAlias[1].arraySetSize(number_of_joints_/*-1*/);
        for(int i=0;i<number_of_joints_;i++)
        {
            commandAlias[1][i] = string("Device/SubDeviceList/"+joint_names_[i]+"/Hardness/Actuator/Value");
        }
        dcm_proxy_->callVoid("createAlias",commandAlias);

    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not initialize dcm aliases!\n\tTrace: %s",e.what());
        return false;
    }

    stiffnesses_enabled_ = true;

    m_jointState.name = m_motionProxy->getBodyNames("Body");

    std::stringstream ss;
    std::copy(m_jointState.name.begin(), m_jointState.name.end()-1, std::ostream_iterator<std::string>(ss,","));
    std::copy(m_jointState.name.end()-1, m_jointState.name.end(), std::ostream_iterator<std::string>(ss));
    ROS_INFO("Romeo joints found: %s",ss.str().c_str());

    return true;
}

bool Romeo::initializeControllers(controller_manager::ControllerManager& cm)
{
    if(!initialize())
    {
        ROS_ERROR("Initialization method failed!");
        return false;
    }

    // Initialize Controllers' Interfaces
    joint_angles_.resize(number_of_joints_);
    joint_velocities_.resize(number_of_joints_);
    joint_efforts_.resize(number_of_joints_);
    joint_commands_.resize(number_of_joints_);

    try
    {
        for(int i=0;i<number_of_joints_;i++)
        {
            hardware_interface::JointStateHandle state_handle(joint_names_[i], &joint_angles_[i],
                                                              &joint_velocities_[i], &joint_efforts_[i]);
            jnt_state_interface_.registerHandle(state_handle);

            hardware_interface::JointHandle pos_handle(jnt_state_interface_.getHandle(joint_names_[i]),
                                                       &joint_commands_[i]);
            jnt_pos_interface_.registerHandle(pos_handle);
        }

        registerInterface(&jnt_state_interface_);
        registerInterface(&jnt_pos_interface_);
    }
    catch(const ros::Exception& e)
    {
        ROS_ERROR("Could not initialize hardware interfaces!\n\tTrace: %s",e.what());
        return false;
    }
    ROS_INFO("Romeo Module initialized!");
    return true;
}

bool Romeo::connect(const ros::NodeHandle nh)
{
    // Initialize ROS nodes
    node_handle_ = nh;

    is_connected_ = false;

    // Load ROS Parameters
    loadParams();

    // Needed for Error Checking
    try
    {
        subscribeToMicroEvent("ClientDisconnected", "Romeo", "brokerDisconnected");
    }
    catch (const AL::ALError& e)
    {
        ROS_ERROR("Could not subscribe to brokerDisconnected!\n\tTrace: %s",e.what());
    }

    // Initialize DCM_motion Proxy
    try
    {
      dcm_proxy_  = boost::shared_ptr<AL::ALProxy>(new AL::ALProxy(getParentBroker(), "DCM_motion"));

    }
    catch (const AL::ALError& e)
    {
        ROS_ERROR("Failed to connect to DCM Proxy!\n\tTrace: %s",e.what());
        return false;
    }

    // Initialize Memory Proxy
    try
    {
        memory_proxy_ = AL::ALMemoryProxy(getParentBroker());
    }
    catch (const AL::ALError& e)
    {
        ROS_ERROR("Failed to connect to Memory Proxy!\n\tTrace: %s",e.what());
        return false;
    }
    try
    {
       m_motionProxy = boost::shared_ptr<AL::ALMotionProxy>(new AL::ALMotionProxy(getParentBroker()));
    }
    catch (const AL::ALError& e)
    {
       ROS_ERROR("Could not create ALMotionProxy.");
       return false;
    }

    is_connected_ = true;

    // Subscribe/Publish ROS Topics/Services
    subscribe();

    // Initialize Controller Manager and Controllers
    manager_ = new controller_manager::ControllerManager(this,node_handle_);
    if(!initializeControllers(*manager_))
    {
        ROS_ERROR("Could not load controllers!");
        return false;
    }
    ROS_INFO("Controllers successfully loaded!");
    return true;
}

void Romeo::disconnect()
{
    if(!is_connected_)
        return;
    try
    {
        unsubscribeFromMicroEvent("ClientDisconnected", "Romeo");
    }
    catch (const AL::ALError& e)
    {
        ROS_ERROR("Failed to unsubscribe from subscribed events!\n\tTrace: %s",e.what());
    }
    is_connected_ = false;
}

void Romeo::subscribe()
{
    // Subscribe/Publish ROS Topics/Services
    cmd_vel_sub_ = node_handle_.subscribe(prefix_+"cmd_vel", topic_queue_, &Romeo::commandVelocity, this);

    stiffness_pub_ = node_handle_.advertise<std_msgs::Float32>(prefix_+"stiffnesses", topic_queue_);
    stiffness_.data = 1.0f;
/*
    //ROS service to enable robot's stiffness, currently not used.
    stiffness_switch_ = node_handle_.advertiseService<Romeo, romeo_dcm_msgs::BoolService::Request,
            romeo_dcm_msgs::BoolService::Response>(prefix_+"Stiffnesses/Enable", &Romeo::switchStiffnesses, this);
*/
    m_jointStatePub = node_handle_.advertise<sensor_msgs::JointState>("joint_states",5);
}

void Romeo::loadParams()
{
    ros::NodeHandle n_p("~");
    // Load Server Parameters
    n_p.param("Version", version_, string("V4"));
    n_p.param("BodyType", body_type_, string("H21"));

    n_p.param("TopicQueue", topic_queue_, 50);

    n_p.param("Prefix", prefix_, string("romeo_dcm"));
    prefix_ = prefix_+"/";

    n_p.param("LowCommunicationFrequency", low_freq_, 10.0);
    n_p.param("HighCommunicationFrequency", high_freq_, 10.0);
    n_p.param("ControllerFrequency", controller_freq_, 15.0);
    n_p.param("JointPrecision", joint_precision_, 0.0174532925);
    n_p.param("OdomFrame", odom_frame_, string("odom"));
}

void Romeo::brokerDisconnected(const string& event_name, const string &broker_name, const string& subscriber_identifier)
{
    if(broker_name == "Romeo Driver Broker")
        is_connected_ = false;
}

void Romeo::DCMTimedCommand(const string &key, const AL::ALValue &value, const int &timeOffset, const string &type)
{
    try
    {
        // Create timed-command
        AL::ALValue command;
        command.arraySetSize(3);
        command[0] = key;
        command[1] = type;
        command[2].arraySetSize(1);
        command[2][0].arraySetSize(2);
        command[2][0][0] = value;
        command[2][0][1] = dcm_proxy_->call<int>("getTime",timeOffset);

        // Execute timed-command
        dcm_proxy_->callVoid("set",command);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not execute DCM timed-command!\n\t%s\n\n\tTrace: %s", key.c_str(), e.what());
    }
}

void Romeo::DCMAliasTimedCommand(const string &alias, const vector<float> &values, const vector<int> &timeOffsets,
                               const string &type, const string &type2)
{
    try
    {
        // Create Alias timed-command
        AL::ALValue command;
        command.arraySetSize(4);
        command[0] = alias;
        command[1] = type;
        command[2] = type2;
        command[3].arraySetSize(values.size());
        int T = dcm_proxy_->call<int>("getTime",0);
        for(int i=0;i<values.size();i++)
        {
            command[3][i].arraySetSize(1);
            command[3][i][0].arraySetSize(2);
            command[3][i][0][0] = values[i];
            command[3][i][0][1] = T+timeOffsets[i];
        }

        // Execute Alias timed-command
        dcm_proxy_->callVoid("setAlias",command);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not execute DCM timed-command!\n\t%s\n\n\tTrace: %s", alias.c_str(), e.what());
    }
}

void Romeo::insertDataToMemory(const string &key, const AL::ALValue &value)
{
    memory_proxy_.insertData(key,value);
}

AL::ALValue Romeo::getDataFromMemory(const string &key)
{
    return memory_proxy_.getData(key);
}

void Romeo::subscribeToEvent(const string &name, const string &callback_module, const string &callback_method)
{
    try
    {
        memory_proxy_.subscribeToEvent(name,callback_module,"",callback_method);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not subscribe to event '%s'.\n\tTrace: %s",name.c_str(),e.what());
    }
}

void Romeo::subscribeToMicroEvent(const string &name, const string &callback_module,
                                const string &callback_method, const string &callback_message)
{
    try
    {
        memory_proxy_.subscribeToMicroEvent(name,callback_module,callback_message,callback_method);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not subscribe to micro-event '%s'.\n\tTrace: %s",name.c_str(),e.what());
    }
}

void Romeo::unsubscribeFromEvent(const string &name, const string &callback_module)
{
    try
    {
        memory_proxy_.unsubscribeToEvent(name,callback_module);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not unsubscribe from event '%s'.\n\tTrace: %s",name.c_str(),e.what());
    }
}

void Romeo::unsubscribeFromMicroEvent(const string &name, const string &callback_module)
{
    try
    {
        memory_proxy_.unsubscribeToMicroEvent(name,callback_module);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not unsubscribe from micro-event '%s'.\n\tTrace: %s",name.c_str(),e.what());
    }
}

void Romeo::raiseEvent(const string &name, const AL::ALValue &value)
{
    try
    {
        memory_proxy_.raiseEvent(name,value);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not raise event '%s'.\n\tTrace: %s",name.c_str(),e.what());
    }
}

void Romeo::raiseMicroEvent(const string &name, const AL::ALValue &value)
{
    try
    {
        memory_proxy_.raiseMicroEvent(name,value);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not raise micro-event '%s'.\n\tTrace: %s",name.c_str(),e.what());
    }
}

void Romeo::declareEvent(const string &name)
{
    try
    {
        memory_proxy_.declareEvent(name);
    }
    catch(AL::ALError& e)
    {
        ROS_ERROR("Could not declare event '%s'.\n\tTrace: %s",name.c_str(),e.what());
    }
}

void Romeo::run()
{
    boost::thread t1(&Romeo::controllerLoop,this);
//    boost::thread t2(&Romeo::lowCommunicationLoop,this);
//    boost::thread t3(&Romeo::highCommunicationLoop,this);
    t1.join();
//    t2.join();
//    t3.join();
}

void Romeo::lowCommunicationLoop()
{
    static ros::Rate rate(low_freq_);
    while(ros::ok())
    {
        ros::Time time = ros::Time::now();

        if(!is_connected_)
            break;

        if(stiffnesses_enabled_)
        {
            stiffness_.data = 1.0f;
        }
        else
        {
            stiffness_.data = 0.0f;
        }
        stiffness_pub_.publish(stiffness_);

        rate.sleep();
    }
}

void Romeo::highCommunicationLoop()
{
    static ros::Rate rate(high_freq_);
    while(ros::ok())
    {
        ros::Time time = ros::Time::now();

        if(!is_connected_)
            break;

	//You can add publisher for sensors value here

        try
        {
            dcm_proxy_->callVoid("ping");
        }
        catch(const AL::ALError& e)
        {
            ROS_ERROR("Could not ping DCM proxy.\n\tTrace: %s",e.what());
            is_connected_ = false;
        }
        rate.sleep();
    }
}

void Romeo::controllerLoop()
{
    static ros::Rate rate(controller_freq_+10.0);
    while(ros::ok())
    {

	
        ros::Time time = ros::Time::now();

        if(!is_connected_)
            break;

	//You can add publisher for sensors value here

        try
        {
            dcm_proxy_->callVoid("ping");
        }
        catch(const AL::ALError& e)
        {
            ROS_ERROR("Could not ping DCM proxy.\n\tTrace: %s",e.what());
            is_connected_ = false;
	    rate.sleep();
	    continue;
        }

        readJoints();

        manager_->update(time,ros::Duration(1.0f/controller_freq_));

        writeJoints();

        publishJointStateFromAlMotion();

        rate.sleep();
    }
}

bool Romeo::connected()
{
    return is_connected_;
}

void Romeo::commandVelocity(const geometry_msgs::TwistConstPtr &msg)
{
    ROS_WARN("This function does nothing at the moment..");
}

void Romeo::readJoints()
{
    vector<float> jointData;
    try
    {
        jointData = memory_proxy_.getListData(joints_names_);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not get joint data from Romeo.\n\tTrace: %s",e.what());
        return;
    }

    for(short i = 0; i<jointData.size(); i++)
    {
        joint_angles_[i] = jointData[i];
        // Set commands to the read angles for when no command specified
        joint_commands_[i] = jointData[i];
    }


}

void Romeo::publishJointStateFromAlMotion(){
  std::vector<float> positionData;
  positionData = m_motionProxy->getAngles("Body", true);
  m_jointState.header.stamp = ros::Time::now();
  m_jointState.header.frame_id = "base_link";
  m_jointState.position.resize(positionData.size());
  for(unsigned i = 0; i<positionData.size(); ++i)
  {
     m_jointState.position[i] = positionData[i];
  }

  m_jointStatePub.publish(m_jointState);
}

void Romeo::writeJoints()
{
    // Update joints only when actual command is issued
    bool changed = false;
    for(int i=0;i<number_of_joints_;i++)
    {
        if(fabs(joint_commands_[i]-joint_angles_[i])>joint_precision_)
        {
            changed = true;
            break;
        }
    }
    // Do not write joints if no change in joint values
    if(!changed)
    {
        return;
    }

    try
    {
        int T = dcm_proxy_->call<int>("getTime",0);
        for(int i=0;i<number_of_joints_;i++)
        {
            commands_[3][i][0][0] = float(joint_commands_[i]);
            commands_[3][i][0][1] = T+(int)(800.0f/controller_freq_);
       //ROS_INFO(<<i<<"\t"<<commands_[3][i][0][0]<<"\n");
        }
        
        dcm_proxy_->callVoid("setAlias",commands_);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not send joint commands to Romeo.\n\tTrace: %s",e.what());
        return;
    }
}


bool Romeo::switchStiffnesses(romeo_dcm_msgs::BoolService::Request &req, romeo_dcm_msgs::BoolService::Response &res)
{
    if(stiffnesses_enabled_!=req.enable && req.enable)
    {
        DCMAliasTimedCommand("JointsHardness",vector<float>(number_of_joints_,1.0f), vector<int>(number_of_joints_,0));
    }
    else if(stiffnesses_enabled_!=req.enable)
    {
        DCMAliasTimedCommand("JointsHardness",vector<float>(number_of_joints_,0.0f), vector<int>(number_of_joints_,0));
    }
    stiffnesses_enabled_ = req.enable;
}


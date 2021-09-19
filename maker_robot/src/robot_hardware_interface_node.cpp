#include <maker_robot/robot_hardware_interface.h>

//namesapce i2c_ros

ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    init();

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);

    right_client = nh_.serviceClient<three_dof_planar_manipulator::Floats_array>("right_joint_states");
    left_client  = nh_.serviceClient<three_dof_planar_manipulator::Floats_array>("left_joint_states");

    
    
    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

void ROBOTHardwareInterface::init() {
	
	for(int i=0; i< JOINT_NUM; i++)
	{
	// Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);
       
    // Create velocity joint interface
	    hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
        velocity_joint_interface_.registerHandle(jointVelocityHandle);

    // Create Joint Limit interface   
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::getJointLimits(joint_name_[i], nh_, limits);
	    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandle(jointVelocityHandle, limits);
	    velocityJointSaturationInterface.registerHandle(jointLimitsHandle);

	}

  
    
// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&velocityJointSaturationInterface);
}

void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ROBOTHardwareInterface::read() {

    uint8_t rbuff[1];
    int x;

   right_joint_read.request.req=1.0;
   left_joint_read.request.req=1.0;

/*
   // left_motor.readBytes(rbuff,1);
    x=0;//(int8_t)rbuff[0];
    left_motor_pos+=angles::from_degrees((double)x);
    joint_position_[0]=left_motor_pos;

    right_motor.readBytes(rbuff,1);
    x=(int8_t)rbuff[0];
    right_motor_pos+=angles::from_degrees((double)x);
    joint_position_[1]=right_motor_pos;
*/
    //ROS_INFO("pos=%.2f x=%d ",pos,x);

   

    if(right_client.call(right_joint_read))
	{
	   // joint_right_position  = angles::from_degrees(right_joint_read.response.res[0]);
	   // joint_left_velocity   = angles::from_degrees(right_joint_read.response.res[1]);

            right_motor_pos += angles::from_degrees((double)right_joint_read.response.res[0]); 
            joint_position_[2]=right_motor_pos ; 
            joint_position_[3]=right_motor_pos ; 
	     
            ROS_INFO("Right Pos: %.2f,  %.2f  ",right_joint_read.response.res[0],angles::from_degrees(right_joint_read.response.res[0]));
          //  right_motor_pos+=joint_right_position;
          //  joint_position_[0] = right_motor_pos;
          //  joint_position_[1] = joint_right_velocity; 

/*
if more than one joint,
        get values for joint_position_2, joint_velocity_2,......
*/	    
	    
	}
	else
	{
	   // joint_right_position  = 0;
	   // joint_left_position   = 0;
	}


    if(left_client.call(left_joint_read))
	{
	  //  joint_right_position  =  angles::from_degrees(left_joint_read.response.res[0]);
	   // joint_left_velocity   =  angles::from_degrees(left_joint_read.response.res[1]);

              left_motor_pos += angles::from_degrees((double)right_joint_read.response.res[0]); 
            joint_position_[0]= left_motor_pos; 
            joint_position_[1]= left_motor_pos;         
	    
            ROS_INFO("Left Pos: %.2f,  %.2f ",left_joint_read.response.res[0],left_joint_read.response.res[1]);

         //   joint_position_[0] = joint_left_position;
         //   joint_position_[1] = joint_left_velocity; 
//
	    
	    
	}
	else
	{
	   // joint_right_position  = 0;
	   // joint_left_position   = 0;
	}
	
}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
   
    velocityJointSaturationInterface.enforceLimits(elapsed_time);   

    
	uint8_t wbuff[2];

    int velocity,result;
    
    
    velocity=(int)angles::to_degrees(joint_velocity_command_[0]);
	wbuff[0]=velocity;
    wbuff[1]=velocity >> 8;
	//ROS_INFO("joint_velocity_command_[0]=%.2f velocity=%d  B1=%d B2=%d", joint_velocity_command_[0],velocity,wbuff[0],wbuff[1]);

    if(left_prev_cmd!=velocity)
    {
	    result = 0 ;// left_motor.writeData(wbuff,2);
	    //ROS_INFO("Writen successfully result=%d", result);
	    left_prev_cmd=velocity;
            ROS_INFO("write left wheel ");
    }
    
    velocity=(int)angles::to_degrees(joint_velocity_command_[1]);
	wbuff[0]=velocity;
    wbuff[1]=velocity >> 8;
	//ROS_INFO("joint_velocity_command_[0]=%.2f velocity=%d  B1=%d B2=%d", joint_velocity_command_[0],velocity,wbuff[0],wbuff[1]);

    if(right_prev_cmd!=velocity)
    {
	    result = 0; // right_motor.writeData(wbuff,2);
	    //ROS_INFO("Writen successfully result=%d", result);
	    right_prev_cmd=velocity;
           ROS_INFO("write left wheel ");
    }

		
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "mobile_robot_hardware_interface");
    ros::NodeHandle nh;
    //ros::AsyncSpinner spinner(4);  
    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    ROBOTHardwareInterface ROBOT(nh);
    //spinner.start();
    spinner.spin();
    //ros::spin();
    return 0;
}

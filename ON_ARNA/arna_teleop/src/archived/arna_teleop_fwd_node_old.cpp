#include <cstdlib>
#include <iostream>
#include <cstdint>
#include <vector>
#include <stdio.h>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Joy.h>

#include <kortex_driver/TwistCommand.h>

#include <kortex_driver/SetCartesianReferenceFrame.h>

#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/GripperMode.h>

#include <kortex_driver/CreateProtectionZone.h>


const std::string arm_fwd_topic("/arm/in/cartesian_velocity");
const std::string base_fwd_topic("/phy_joy_topic");

//globals
int ticks_since_msg = 0;
int recv_first_msg = 0;
std::vector<float> mov_vector(10, 0);

void mov_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    mov_vector = msg->data;
    ticks_since_msg = 0;
    recv_first_msg = 1;
}

int main(int argc, char** argv){

    std::cout << "begin" <<std::endl;
    ros::init(argc, argv, "ARNA_TELEOP_FWD_NODE");
    ros::NodeHandle n;

	sleep(10);

	//configure arm
    //set cartesian reference
	kortex_driver::SetCartesianReferenceFrame cartref;
	cartref.request.input.reference_frame = 2;
	ros::service::call("/arm/control_config/set_cartesian_reference_frame", cartref);

    /*
    //apply safety zones - safeteyzones are persistent across restarts so I think this is unescissary
    //apparently list initialization doesnt work on these
    kortex_driver::Base_RotationMatrix rm{};
    rm.row1.column1 = 1; rm.row1.column2 = 0; rm.row1.column3 = 0;
    rm.row2.column1 = 0; rm.row2.column2 = 1; rm.row2.column3 = 0;
    rm.row3.column1 = 0; rm.row3.column2 = 0; rm.row3.column3 = 1;
    
    //riser zone
    kortex_driver::ZoneShape zs1{};
    zs1.shape_type = 3;
    zs1.origin.x = 0; zs1.origin.y = -0.1; zs1.origin.z = -0.2;
    zs1.orientation = rm;
    zs1.dimensions = {0.3, 0.5, 0.6};
    kortex_driver::ProtectionZone pz1{};
    pz1.name = "RISER PROTECTION ZONE";
    pz1.is_enabled = 1;
    pz1.shape = zs1;
    kortex_driver::CreateProtectionZone cpz1{};
    cpz1.request.input = pz1;
    ros::service::call("/arm/base/create_protection_zone", cpz1);
    */

    //base
    ros::Subscriber sub_base = n.subscribe("ARNA_TELEOP_MOV", 100, mov_callback);
    
    //arm
    ros::Publisher pub_arm = n.advertise<kortex_driver::TwistCommand>(arm_fwd_topic, 100);
    ros::Publisher pub_base = n.advertise<sensor_msgs::Joy>(base_fwd_topic, 100);
    ros::ServiceClient srv_gripper = n.serviceClient<kortex_driver::SendGripperCommand>("/arm/base/send_gripper_command");


    //wait until first message received
    while(recv_first_msg == 0 && ros::ok()){
        ros::spinOnce();
        usleep(50000);
    }

    //main loop: spin, send joy msg, increment tick counter, check if DC
    while(ros::ok()){
        
        ros::spinOnce();

        sensor_msgs::Joy base_msg;
        base_msg.axes = std::vector<float>({mov_vector[0], mov_vector[1], mov_vector[2]}); //this only works because msg.axes is a std::vector
        pub_base.publish(base_msg);

        kortex_driver::TwistCommand arm_msg;
        arm_msg.reference_frame = 0;
        arm_msg.twist.linear_x = mov_vector[3];
        arm_msg.twist.linear_y = mov_vector[4];
        arm_msg.twist.linear_z = mov_vector[5];
        arm_msg.twist.angular_x = mov_vector[6];
        arm_msg.twist.angular_y = mov_vector[7];
        arm_msg.twist.angular_z = mov_vector[8];
        arm_msg.duration = 0;
        pub_arm.publish(arm_msg);

        kortex_driver::SendGripperCommand gripper_msg;
        kortex_driver::Finger finger;
        finger.finger_identifier = 0;
        finger.value = mov_vector[9];
        gripper_msg.request.input.gripper.finger.push_back(finger);
        gripper_msg.request.input.mode = kortex_driver::GripperMode::GRIPPER_POSITION;
        srv_gripper.call(gripper_msg);

        usleep(50000); 
        
        //if 100ms since last msg zero arm/base movement vector
        ticks_since_msg++;       
        if(ticks_since_msg > 10){
            std::cout << "100ms since last base message. Zeroing movement." << std::endl;
            std::fill(mov_vector.begin(), mov_vector.end(), 0);
        }
    }

   std::cout << "terminated" << std::endl;
}


#include <cmath>
#include <cstdlib>
#include <iostream>
#include <cstdint>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <fstream>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <kortex_driver/TwistCommand.h>
#include <kortex_driver/SetCartesianReferenceFrame.h>
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/GripperMode.h>
#include <kortex_driver/CreateProtectionZone.h>

#include <opencv2/opencv.hpp>

const std::string arm_fwd_topic("/arm/in/cartesian_velocity");
const std::string base_fwd_topic("/phy_joy_topic");
const std::string depth_fwd_topic("/ARNA_TELEOP_DEPTH");
const std::string depth_camera_topic("/arm_cam/depth/image_rect");

//globals
int ticks_since_msg = 0;
int recv_first_msg = 0;
std::vector<float> button_mov_vector(10, 0);
float depth;

void depth_callback(const sensor_msgs::ImageConstPtr& msg){

    //decompress image
    int data_size = msg->data.size();
    int height = msg->height;
    int width = msg->width;
    cv::Mat image(cv::Size(height, width), CV_32FC1, (void*)msg->data.data());

    //compute average depth of circle around center
    int center_x = image.cols / 2;
    int center_y = image.rows / 2;
    int radius = 25;  
    float sum = 0.0;
    int count = 0;

    for (int y = std::max(0, center_y - radius); y < std::min(image.rows, center_y + radius); ++y) {
        for (int x = std::max(0, center_x - radius); x < std::min(image.cols, center_x + radius); ++x) {
            if ((x - center_x)*(x - center_x) + (y - center_y)*(y - center_y) <= radius * radius) {
                float val = image.at<float>(y, x);
                if(!std::isnan(val)){
                    sum += val;
                    ++count;
                }
            }
        }
    }

    depth = sum / count;
}

void mov_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    button_mov_vector = msg->data;
    ticks_since_msg = 0;
    recv_first_msg = 1;
}

int main(int argc, char** argv){

    std::cout << "begin" <<std::endl;
    ros::init(argc, argv, "ARNA_TELEOP_FWD_NODE");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

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

    //control
    ros::Subscriber sub_base = n.subscribe("ARNA_TELEOP_MOV", 100, mov_callback);
    ros::Publisher pub_arm = n.advertise<kortex_driver::TwistCommand>(arm_fwd_topic, 100);
    ros::Publisher pub_base = n.advertise<sensor_msgs::Joy>(base_fwd_topic, 100);
    ros::ServiceClient srv_gripper = n.serviceClient<kortex_driver::SendGripperCommand>("/arm/base/send_gripper_command");

    //depth
    image_transport::Subscriber sub_depth = it.subscribe(depth_camera_topic, 1, depth_callback);
    ros::Publisher pub_depth = n.advertise<std_msgs::Float32>(depth_fwd_topic, 100);

    //wait until first message received
    while(recv_first_msg == 0 && ros::ok()){
        ros::spinOnce();
        usleep(50000);
    }

    //main loop: spin, send joy msg, increment tick counter, check if DC
    while(ros::ok()){
        
        ros::spinOnce();

        sensor_msgs::Joy base_msg;
        base_msg.axes = std::vector<float>({button_mov_vector[0], button_mov_vector[1], button_mov_vector[2]}); //this only works because msg.axes is a std::vector
        pub_base.publish(base_msg);

        kortex_driver::TwistCommand arm_msg;
        arm_msg.reference_frame = 0;
        arm_msg.twist.linear_x = button_mov_vector[3];
        arm_msg.twist.linear_y = button_mov_vector[4];
        arm_msg.twist.linear_z = button_mov_vector[5];
        arm_msg.twist.angular_x = button_mov_vector[6];
        arm_msg.twist.angular_y = button_mov_vector[7];
        arm_msg.twist.angular_z = button_mov_vector[8];
        arm_msg.duration = 0;
        pub_arm.publish(arm_msg);

        kortex_driver::SendGripperCommand gripper_msg;
        kortex_driver::Finger finger;
        finger.finger_identifier = 0;
        finger.value = button_mov_vector[9];
        gripper_msg.request.input.gripper.finger.push_back(finger);
        gripper_msg.request.input.mode = kortex_driver::GripperMode::GRIPPER_POSITION;
        srv_gripper.call(gripper_msg);

        std_msgs::Float32 depth_msg;
        depth_msg.data = depth;
        pub_depth.publish(depth_msg);

        usleep(50000); 
        
        //if 100ms since last msg zero arm/base movement vector
        ticks_since_msg++;       
        if(ticks_since_msg > 10){
            std::cout << "100ms since last base message. Zeroing movement." << std::endl;
            std::fill(button_mov_vector.begin(), button_mov_vector.end(), 0);
        }
    }

   std::cout << "terminated" << std::endl;
}


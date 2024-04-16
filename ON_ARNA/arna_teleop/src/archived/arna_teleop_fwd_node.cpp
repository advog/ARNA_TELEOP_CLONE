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
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <image_transport/image_transport.h>

#include <kortex_driver/TwistCommand.h>
#include <kortex_driver/SetCartesianReferenceFrame.h>
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/GripperMode.h>
#include <kortex_driver/CreateProtectionZone.h>
#include <kortex_driver/Base_ClearFaults.h>
#include <kortex_driver/ReadAction.h>
#include <kortex_driver/ExecuteAction.h>
#include <kortex_driver/ActionNotification.h>
#include <kortex_driver/ActionEvent.h>
#include <kortex_driver/ActionType.h>
#include <kortex_driver/OnNotificationActionTopic.h>

#include <opencv2/opencv.hpp>

const int HOME_ACTION_IDENTIFIER = 2;

const std::string mov_in_topic("ARNA_TELEOP_MOV");
const std::string command_in_topic("ARNA_TELEOP_COM");
const std::string depth_camera_in_topic("/arm_cam/depth/image_rect");

const std::string arm_out_topic("/arm/in/cartesian_velocity");
const std::string base_out_topic("/cmd_vel");
const std::string depth_out_topic("/ARNA_TELEOP_DEPTH");
const std::string gripper_out_topic("/arm/base/send_gripper_command");

//globals
int ticks_since_msg = 0;
int recv_first_msg = 0;
std::vector<float> mov_vector(10, 0);
std::int32_t command = 0;
std::int32_t last_action_notification_event = 0;
float depth = 0;

//
// Callbacks
//

void command_callback(const std_msgs::Int32::ConstPtr& msg){
    command = msg->data;
}

void notification_callback(const kortex_driver::ActionNotification& notif){
    last_action_notification_event = notif.action_event;
}

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

	//update depth
    depth = sum / count;
}

void mov_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    
    //copy move vector
    mov_vector = msg->data;

    //indicate start of connection
    ticks_since_msg = 0;
    recv_first_msg = 1;
}

//
// Command Execution
//

bool wait_for_action_end_or_abort()
{
  while (ros::ok())
  {
    if (last_action_notification_event == kortex_driver::ActionEvent::ACTION_END)
    {
      return true;
    }
    else if (last_action_notification_event == kortex_driver::ActionEvent::ACTION_ABORT)
    {
      return false;
    }
    ros::spinOnce();
  }
  return false;
}

bool execute_home_arm(ros::NodeHandle& n){

    ros::ServiceClient service_client_read_action = n.serviceClient<kortex_driver::ReadAction>("/arm/base/read_action");
    kortex_driver::ReadAction service_read_action;
    last_action_notification_event = 0;

    // The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
    service_read_action.request.input.identifier = HOME_ACTION_IDENTIFIER;

    if (!service_client_read_action.call(service_read_action)){
        std::string error_string = "Failed to call ReadAction";
        return false;
    }

    // We can now execute the Action that we read 
    ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/arm/base/execute_action");
    kortex_driver::ExecuteAction service_execute_action;
    service_execute_action.request.input = service_read_action.response.output;

    if (!service_client_execute_action.call(service_execute_action)){
        std::string error_string = "Failed to call ExecuteAction";
        return false;
    }

    return wait_for_action_end_or_abort();
}

void execute_command(ros::NodeHandle& n){
    if(command == HOME_ACTION_IDENTIFIER){
        execute_home_arm(n);
    }
    command = 0;
}

//
// Movement Execution
//

float base_t_scale(float in){
	
    //scale joystick input
    float abs_in_max = 300;
	float abs_out_max = 0.1;
	return (in/abs_in_max)*abs_out_max;
}

float base_r_scale(float in){

    //scale joystick input
	float abs_in_max = 300;
	float abs_out_max = 0.05;
	return (in/abs_in_max)*abs_out_max;
}

//
// main
//

int main(int argc, char** argv){

	//init
    std::cout << "begin" <<std::endl;
    ros::init(argc, argv, "ARNA_TELEOP_FWD_NODE");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

	//configure arm
    //set cartesian reference
	kortex_driver::SetCartesianReferenceFrame cartref;
	cartref.request.input.reference_frame = 2;
	ros::service::call("/arm/control_config/set_cartesian_reference_frame", cartref);

    //control pub subs
    ros::Subscriber sub_base = n.subscribe(mov_in_topic, 100, mov_callback);
    ros::Publisher pub_arm = n.advertise<kortex_driver::TwistCommand>(arm_out_topic, 10);
    
    //pub for vel base
    //ros::Publisher pub_base = n.advertise<geometry_msgs::Twist>(base_out_topic, 10);

    //pub for joy base
    ros::Publisher pub_base = n.advertise<sensor_msgs::Jot>(base_out_topic, 10);

    ros::ServiceClient srv_gripper = n.serviceClient<kortex_driver::SendGripperCommand>(gripper_out_topic);
    
    //depth pub sub
    image_transport::Subscriber sub_depth = it.subscribe(depth_camera_in_topic, 1, depth_callback);
    ros::Publisher pub_depth = n.advertise<std_msgs::Float32>(depth_out_topic, 10);

    //command sub
    ros::Subscriber sub_command = n.subscribe(command_in_topic, 10, command_callback);

    //arm action service
    ros::ServiceClient service_client_activate_notif = n.serviceClient<kortex_driver::OnNotificationActionTopic>("/arm/base/activate_publishing_of_action_topic");
    kortex_driver::OnNotificationActionTopic service_activate_notif;
    service_client_activate_notif.call(service_activate_notif);
    ros::Duration(1.00).sleep();
    ros::Subscriber sub = n.subscribe("/arm/action_topic", 100, notification_callback);

    //wait until first message received
    while(recv_first_msg == 0 && ros::ok()){
        ros::spinOnce();
        usleep(50000);
    }

    //main loop
    while(ros::ok()){
        
        ros::spinOnce();

        //if command received, overide movement and execute command
        if(command != 0){
            execute_command(n);
        }
        //else execute movement
        else{
            //execute_movement(n);
            //publish base command
            geometry_msgs::Twist base_msg;
            base_msg.linear.x = base_t_scale(mov_vector[1]);
            base_msg.linear.y = -base_t_scale(mov_vector[0]);
            base_msg.angular.z = -base_r_scale(mov_vector[2]);
            pub_base.publish(base_msg);

            //publish arm command
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
            
            //publish gripper command
            kortex_driver::SendGripperCommand gripper_msg;
            kortex_driver::Finger finger;
            finger.finger_identifier = 0;
            finger.value = mov_vector[9];
            gripper_msg.request.input.gripper.finger.push_back(finger);
            gripper_msg.request.input.mode = kortex_driver::GripperMode::GRIPPER_POSITION;
            srv_gripper.call(gripper_msg);

            //publish depth value
            std_msgs::Float32 depth_msg;
            depth_msg.data = depth;
            pub_depth.publish(depth_msg);
        }

        usleep(50000); 
        
        //if 500ms since last msg zero arm/base movement vector
        ticks_since_msg++;       
        if(ticks_since_msg > 10){
            std::cout << "100ms since last base message. Zeroing movement." << std::endl;
            std::fill(mov_vector.begin(), mov_vector.end(), 0);
        }
    }

   std::cout << "terminated" << std::endl;
}


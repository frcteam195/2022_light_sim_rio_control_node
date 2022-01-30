#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#include "rio_control_node/Motor_Control.h"
#include "rio_control_node/Motor_Configuration.h"
#include "rio_control_node/Motor_Status.h"
#include "rio_control_node/Robot_Status.h"
#include "rio_control_node/Joystick_Status.h"

#include <thread>
#include <string>
#include <mutex>

#define RATE (100)

ros::NodeHandle* node;

// void gazebo_link_states_callback(const gazebo_msgs::LinkStates &msg)
// {
// 	static ros::Publisher motor_status_pub
//         = node->advertise<rio_control_node::Motor_Status>("MotorStatus", 1);

//     rio_control_node::Motor_Status motor_status;

//     for( size_t i = 0; i < msg.name.size(); i++)
//     {
//         std::string link_name = msg.name[i];
//         // we are watching this link
//         if( links_to_watch.count( link_name ) )
//         {
//             rio_control_node::Motor_Info motor_info;
//             motor_info.sensor_position = msg.pose[i].position.y;
//             motor_info.sensor_velocity = msg.twist[i].angular.y;
//             motor_info.id = links_to_watch[link_name];
//             motor_status.motors.push_back( motor_info );
//         }
//     }

//     motor_status_pub.publish(motor_status);
// }


void motor_config_callback(const rio_control_node::Motor_Configuration &msg)
{
    (void)msg;
}

static std::map<uint8_t, rio_control_node::Motor_Info> motor_info_map;

void publish_motor_status()
{
    rio_control_node::Motor_Status motor_status;

    for(std::map<uint8_t, rio_control_node::Motor_Info>::iterator i = motor_info_map.begin();
        i != motor_info_map.end();
        i++)
    {
        motor_status.motors.push_back((*i).second);
    }

    static ros::Publisher status_publisher = node->advertise<rio_control_node::Motor_Status>("/MotorStatus", 100);
    status_publisher.publish(motor_status);
}

void motor_control_callback(const rio_control_node::Motor_Control &msg)
{
    double right_side_rpm;
    double left_side_rpm;

    int left_master_id, right_master_id;
    
    double drive_kV;
    
    bool data_verified = true;

    data_verified &= node->getParam("/light_sim_rio_control_node/left_master_id", left_master_id);
    data_verified &= node->getParam("/light_sim_rio_control_node/right_master_id", right_master_id);
    data_verified &= node->getParam("/light_sim_rio_control_node/drive_Kv", drive_kV);

    if (!data_verified)
    {
        ROS_ERROR("Couldn't verify args!");
        return;
    }

    for( std::vector<rio_control_node::Motor>::const_iterator i = msg.motors.begin();
         i != msg.motors.end();
         i++ )
    {
        if ((*i).id == left_master_id)
        {
            if((*i).control_mode == rio_control_node::Motor::PERCENT_OUTPUT)
            {
                left_side_rpm = (*i).output_value / drive_kV;
                rio_control_node::Motor_Info motor_info;
                motor_info.bus_voltage = 12;
                motor_info.id = left_master_id;
                motor_info.sensor_velocity = left_side_rpm;
                motor_info_map[motor_info.id] = motor_info;
            }
        }
        if((*i).id == right_master_id)
        {
            if((*i).control_mode == rio_control_node::Motor::PERCENT_OUTPUT)
            {
               right_side_rpm = (*i).output_value / drive_kV;
                rio_control_node::Motor_Info motor_info;
                motor_info.bus_voltage = 12;
                motor_info.id = right_master_id;
                motor_info.sensor_velocity = right_side_rpm;
                motor_info_map[motor_info.id] = motor_info;
            }
        }
    }

}

void publish_robot_status()
{
    rio_control_node::Robot_Status robot_status;
    robot_status.alliance = rio_control_node::Robot_Status::RED;
    robot_status.robot_state = rio_control_node::Robot_Status::TELEOP;
    static ros::Publisher robot_status_publisher = node->advertise<rio_control_node::Robot_Status>("/RobotStatus", 100);
    robot_status_publisher.publish(robot_status);
}

void linux_joystick_subscriber(const sensor_msgs::Joy &msg)
{
    static ros::Publisher joystick_publisher = node->advertise<rio_control_node::Joystick_Status>("/JoystickStatus", 100);
    
    rio_control_node::Joystick_Status joystick_status;
    rio_control_node::Joystick joystick;

    for(size_t i = 0; i < msg.axes.size(); i++)
    {
        // MGT - it seems like the linux axes are inverted from the DS ones,
        // but this should be double verified
        joystick.axes.push_back(-msg.axes[i]);
    }

    for(std::vector<int>::const_iterator i = msg.buttons.begin();
        i != msg.buttons.end();
        i++)
    {
        joystick.buttons.push_back(*i);
    }

    joystick_status.joysticks.push_back(joystick);
    joystick_publisher.publish(joystick_status);

    publish_robot_status();
    publish_motor_status();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "light_sim_rio_control_node");
	ros::NodeHandle n;
	node = &n;
    ros::Rate rate(RATE);

	ros::Subscriber motorConfig = node->subscribe("/MotorConfiguration", 100, motor_config_callback);
	ros::Subscriber motorControl = node->subscribe("/MotorControl", 100, motor_control_callback);
    ros::Subscriber linux_joystick = node->subscribe("/joy", 100, linux_joystick_subscriber);

    while( ros::ok() )
    {
        ros::spinOnce();
        rate.sleep();
    }


	return 0;
}

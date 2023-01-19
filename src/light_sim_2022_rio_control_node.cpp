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

#include <signal.h>
#include <thread>
#include <string>
#include <mutex>
#include <atomic>

#define RATE (100)

#define STR_PARAM(s) #s
#define CKSP(s) ckgp( STR_PARAM(s) )
std::string ckgp(std::string instr)
{
	std::string retVal = ros::this_node::getName();
	retVal += "/" + instr;
	return retVal;
}

ros::NodeHandle* node;
std::atomic<bool> sigintCalled;
void sigint_handler(int sig)
{
	(void)sig;
	sigintCalled = true;
	ros::shutdown();
}

static std::vector<float> gear_ratio_to_output_shaft;
static std::vector<float> motor_ticks_per_revolution;
static std::vector<float> motor_ticks_velocity_sample_window;

static std::map<uint8_t, rio_control_node::Motor_Config> motor_config_map;
static std::map<uint8_t, rio_control_node::Motor_Info> motor_info_map;

void motor_config_callback(const rio_control_node::Motor_Configuration &msg)
{
    for (auto i = msg.motors.begin(); i != msg.motors.end(); i++)
    {
        // Get the existing data, if it exists.
        rio_control_node::Motor_Config motor_config;
        if (motor_config_map.find((*i).id) != motor_config_map.end())
        {
            motor_config = motor_config_map[(*i).id];
        }

        // Fill out the latest status information.
        motor_config.id = (*i).id;

        motor_config.forward_soft_limit_enable = (*i).forward_soft_limit_enable;
        motor_config.forward_soft_limit = (*i).forward_soft_limit;

        motor_config.reverse_soft_limit_enable = (*i).reverse_soft_limit_enable;
        motor_config.reverse_soft_limit = (*i).reverse_soft_limit;

        motor_config_map[motor_config.id] = motor_config;
    }
}

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

void load_config_params()
{
    bool received_data = false;
    received_data = node->getParam(CKSP(gear_ratio_to_output_shaft), gear_ratio_to_output_shaft);
    if(!received_data)
    {
        ROS_ERROR("COULD NOT LOAD GEAR RATIOS, using 1.0");
        for(uint32_t i = 0;
            i < 20;
            i++)
        {
            gear_ratio_to_output_shaft.push_back(1.0);
        }
    }

    received_data = node->getParam(CKSP(motor_ticks_per_revolution), motor_ticks_per_revolution);
    if(!received_data)
    {
        ROS_ERROR("COULD NOT LOAD ENCODER TICK COUNT, using 2048");
        for(uint32_t i = 0;
            i < 20;
            i++)
        {
            motor_ticks_per_revolution.push_back(2048.0);
        }
    }

    received_data = node->getParam(CKSP(motor_ticks_velocity_sample_window), motor_ticks_velocity_sample_window);
    if(!received_data)
    {
        ROS_ERROR("COULD NOT LOAD ENCODER SAMPLE WINDOW, using 0.1");
        for(uint32_t i = 0;
            i < 20;
            i++)
        {
            motor_ticks_velocity_sample_window.push_back(0.1);
        }
    }
}

void motor_control_callback(const rio_control_node::Motor_Control &msg)
{
    for( std::vector<rio_control_node::Motor>::const_iterator i = msg.motors.begin();
         i != msg.motors.end();
         i++ )
    {
        if ((*i).control_mode == rio_control_node::Motor::PERCENT_OUTPUT)
        {
            // Get the existing data, if it exists.
            rio_control_node::Motor_Info motor_info;
            if (motor_info_map.find((*i).id) != motor_info_map.end())
            {
                motor_info = motor_info_map[(*i).id];
            }

            // Fill out the latest status information.
            motor_info.bus_voltage = 12;
            motor_info.id = (*i).id;
            motor_info.sensor_velocity = (*i).output_value * 6380.0 / gear_ratio_to_output_shaft[(*i).id];
            motor_info.sensor_position += motor_info.sensor_velocity * 0.01;

            if (motor_config_map.find((*i).id) != motor_config_map.end())
            {
                rio_control_node::Motor_Config motor_config = motor_config_map[(*i).id];

                if (motor_config.forward_soft_limit_enable)
                {
                    motor_info.sensor_position = fmin(motor_info.sensor_position, motor_config.forward_soft_limit);
                }

                if (motor_config.reverse_soft_limit_enable)
                {
                    motor_info.sensor_position = fmax(motor_info.sensor_position, motor_config.reverse_soft_limit);
                }
            }

            motor_info_map[motor_info.id] = motor_info;
        }
        else if ((*i).control_mode == rio_control_node::Motor::MOTION_MAGIC)
        {
            float last_position = 0;
            if(motor_info_map.find((*i).id) != motor_info_map.end())
            {
                last_position = motor_info_map[(*i).id].sensor_position;
            }
            rio_control_node::Motor_Info motor_info;
            motor_info.bus_voltage = 12;
            motor_info.id = (*i).id;
            motor_info.sensor_position = ((*i).output_value + (last_position * 5.0)) / 6.0;
            motor_info_map[motor_info.id] = motor_info;
        }
        else if ((*i).control_mode == rio_control_node::Motor::POSITION)
        {
            float last_position = 0;
            if(motor_info_map.find((*i).id) != motor_info_map.end())
            {
                last_position = motor_info_map[(*i).id].sensor_position;
            }
            rio_control_node::Motor_Info motor_info;
            motor_info.bus_voltage = 12;
            motor_info.id = (*i).id;
            motor_info.sensor_position = ((*i).output_value + (last_position * 5.0)) / 6.0;
            motor_info_map[motor_info.id] = motor_info;
        }
    }
}

void publish_robot_status()
{
    static ros::Time start = ros::Time::now();
    rio_control_node::Robot_Status robot_status;
    robot_status.alliance = rio_control_node::Robot_Status::RED;
    if ((ros::Time::now() - start).toSec() > 15.0)
    {
        robot_status.robot_state = rio_control_node::Robot_Status::AUTONOMOUS;
    }
    else
    {
        robot_status.robot_state = rio_control_node::Robot_Status::TELEOP;
    }

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
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "light_sim_rio_control_node");

	sigintCalled = false;
	signal(SIGINT, sigint_handler);

	ros::NodeHandle n;
	node = &n;
    ros::Rate rate(RATE);

    load_config_params();

	ros::Subscriber motorConfig = node->subscribe("/MotorConfiguration", 100, motor_config_callback);
	ros::Subscriber motorControl = node->subscribe("/MotorControl", 100, motor_control_callback);
    ros::Subscriber linux_joystick = node->subscribe("/joy", 100, linux_joystick_subscriber);

    while( ros::ok() )
    {
        ros::spinOnce();
        publish_robot_status();
        publish_motor_status();
        rate.sleep();
    }


	return 0;
}

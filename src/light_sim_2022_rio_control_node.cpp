#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Joy.h"

#include "ck_ros_base_msgs_node/Motor_Control.h"
#include "ck_ros_base_msgs_node/Solenoid_Control.h"
#include "ck_ros_base_msgs_node/Motor_Configuration.h"
#include "ck_ros_base_msgs_node/Motor_Status.h"
#include "ck_ros_base_msgs_node/Robot_Status.h"
#include "ck_ros_base_msgs_node/Joystick_Status.h"
#include "ck_ros_msgs_node/Swerve_Drivetrain_Diagnostics.h"

#include <ck_utilities/Logger.hpp>
#include <ck_utilities/CKMath.hpp>
#include <ck_utilities/geometry/geometry.hpp>
#include <ck_utilities/geometry/geometry_ros_helpers.hpp>

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

static std::map<uint8_t, ck_ros_base_msgs_node::Motor_Config> motor_config_map;
static std::map<uint8_t, ck_ros_base_msgs_node::Motor_Info> motor_info_map;
static std::map<uint8_t, ck_ros_base_msgs_node::Motor> motor_control_map;
static std::map<uint32_t, ck_ros_base_msgs_node::Solenoid> solenoid_control_map;
static float angular_rate_rad_s = 0;

void motor_config_callback(const ck_ros_base_msgs_node::Motor_Configuration &msg)
{
    for (auto i = msg.motors.begin(); i != msg.motors.end(); i++)
    {
        // Get the existing data, if it exists.
        ck_ros_base_msgs_node::Motor_Config motor_config;
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
        motor_config.motion_acceleration = (*i).motion_acceleration;
        motor_config.motion_cruise_velocity = (*i).motion_cruise_velocity;
        motor_config.motion_s_curve_strength = (*i).motion_s_curve_strength;

        motor_config_map[motor_config.id] = motor_config;
    }
}

void publish_imu_data()
{
	static ros::Publisher imu_data_pub = node->advertise<nav_msgs::Odometry>("/RobotIMU", 1);

    {
        nav_msgs::Odometry odometry_data;
        odometry_data.header.stamp = ros::Time::now();
        odometry_data.header.frame_id = "odom";
        odometry_data.child_frame_id = "base_link";

        geometry::Pose empty_pose;
        odometry_data.pose.pose = geometry::to_msg(empty_pose);

        geometry::Twist twist;
        twist.angular.yaw(angular_rate_rad_s);
        odometry_data.twist.twist = geometry::to_msg(twist);

        geometry::Covariance covariance;
        covariance.yaw_var(0.00001);
        odometry_data.twist.covariance = geometry::to_msg(covariance);

        imu_data_pub.publish(odometry_data);
    }
}

void publish_motor_status()
{
    ck_ros_base_msgs_node::Motor_Status motor_status;

    if(solenoid_control_map[65536].output_value == ck_ros_base_msgs_node::Solenoid::OFF)
    {
        motor_info_map[13].reverse_limit_closed = 1;
    }
    else
    {
        motor_info_map[13].reverse_limit_closed = 0;
    }

    for(std::map<uint8_t, ck_ros_base_msgs_node::Motor_Info>::iterator i = motor_info_map.begin();
        i != motor_info_map.end();
        i++)
    {
        motor_status.motors.push_back((*i).second);
    }


    static ros::Publisher status_publisher = node->advertise<ck_ros_base_msgs_node::Motor_Status>("/MotorStatus", 100);
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


void solenoid_control_callback(const ck_ros_base_msgs_node::Solenoid_Control &msg)
{
    for (auto &i : msg.solenoids)
    {
        solenoid_control_map[i.id] = i;
    }
}

void motor_control_callback(const ck_ros_base_msgs_node::Motor_Control &msg)
{
    for( auto &i : msg.motors)
    {
        motor_control_map[i.id] = i;
    }
}

static ck_ros_base_msgs_node::Robot_Status override_robot_status;

void sim_robot_state_subscriber(const ck_ros_base_msgs_node::Robot_Status &msg)
{
    override_robot_status = msg;
}

void publish_robot_status()
{
    static ros::Publisher robot_status_publisher = node->advertise<ck_ros_base_msgs_node::Robot_Status>("/RobotStatus", 100);
    robot_status_publisher.publish(override_robot_status);
}

static ck_ros_base_msgs_node::Joystick_Status joystick_status;

void publish_joystick_status()
{
    static ros::Publisher joystick_publisher = node->advertise<ck_ros_base_msgs_node::Joystick_Status>("/JoystickStatus", 100);

    joystick_publisher.publish(joystick_status);
}

void sim_joystick_subscriber(const ck_ros_base_msgs_node::Joystick_Status &msg)
{
    joystick_status = msg;
}

void swerve_diag_subscriber(const ck_ros_msgs_node::Swerve_Drivetrain_Diagnostics &msg)
{
    angular_rate_rad_s = ck::math::deg2rad(msg.compensated_target_angular_speed_deg_s);
}

void drive_motor_simulation()
{
    static ros::Time prev_time = ros::Time::now();
    ros::Time time_now = ros::Time::now();
    ros::Duration dt = time_now - prev_time;
    prev_time = time_now;
    for( auto &kv : motor_control_map)
    {
        auto i = kv.second;
        if (i.control_mode == ck_ros_base_msgs_node::Motor::PERCENT_OUTPUT)
        {
            // Get the existing data, if it exists.
            ck_ros_base_msgs_node::Motor_Info motor_info;
            if (motor_info_map.find(i.id) != motor_info_map.end())
            {
                motor_info = motor_info_map[i.id];
            }

            // Fill out the latest status information.
            motor_info.bus_voltage = 12;
            motor_info.id = i.id;
            motor_info.sensor_velocity = i.output_value * 6380.0 / gear_ratio_to_output_shaft[i.id - 1];
            motor_info.sensor_position += motor_info.sensor_velocity * 0.01;

            if (motor_config_map.find(i.id) != motor_config_map.end())
            {
                ck_ros_base_msgs_node::Motor_Config motor_config = motor_config_map[i.id];

                if (motor_config.forward_soft_limit_enable)
                {
                    //Come up with a way to take into account rotations offset to 0 on this so min position is not set to limit on boot
                    if (motor_config.forward_soft_limit > 0)
                    {
                        motor_info.sensor_position = fmin(motor_info.sensor_position, motor_config.forward_soft_limit);
                    }
                }

                if (motor_config.reverse_soft_limit_enable)
                {
                    motor_info.sensor_position = fmax(motor_info.sensor_position, motor_config.reverse_soft_limit);
                }
            }

            motor_info_map[motor_info.id] = motor_info;
        }
        else if (i.control_mode == ck_ros_base_msgs_node::Motor::MOTION_MAGIC)
        {
            float last_position = 0;
            float last_velocity = 0;
            if(motor_info_map.find(i.id) != motor_info_map.end())
            {
                last_position = motor_info_map[i.id].sensor_position;
                last_velocity = motor_info_map[i.id].sensor_velocity;
            }

            double cpr = motor_ticks_per_revolution[i.id - 1];
            double gear_ratio = gear_ratio_to_output_shaft[i.id - 1];

            const ck_ros_base_msgs_node::Motor_Config& motor_config = motor_config_map[i.id];
            double accel_rps2 = motor_config.motion_acceleration * 10.0 / cpr / gear_ratio;
            double cruise_vel_rps = motor_config.motion_cruise_velocity * 10.0 / cpr / gear_ratio;
            // double s_curve_strength = motor_config.motion_s_curve_strength;

            double accel_step = accel_rps2 * dt.toSec();

            double curr_setpoint = i.output_value;
            double actual_position = last_position;
            double error = curr_setpoint - actual_position;
            double sign = (double(0) < error) - (error < double(0));


            // we're gonna cheat a bit for now because I'm lazy, and not bother with the decel - MGT
            double curr_velocity = last_velocity + (sign * accel_step);
            if (error > std::numeric_limits<float>::epsilon())
            {
                double decel_limit_velocity = std::sqrt(2*accel_rps2*error);
                curr_velocity = ck::math::limit(curr_velocity, -decel_limit_velocity, decel_limit_velocity);
            }
            curr_velocity = ck::math::limit(curr_velocity, -cruise_vel_rps, cruise_vel_rps);

            double vel_step = curr_velocity * dt.toSec();
            if (std::abs(error) < std::abs(vel_step))
            {
                curr_velocity = 0;
                actual_position = curr_setpoint;
            }
            else
            {
                actual_position = last_position + vel_step;
            }
            // if(i.id == 11){// || i.id == 9){
            //     ck::log_error << "ID: " << i.id << "\tAC: " << actual_position << "\tSP: " << curr_setpoint << " \tCV: " << curr_velocity << "\tAS: " << accel_step << "\tARPS: " << motor_config.motion_acceleration << std::flush;
            // }

            ck_ros_base_msgs_node::Motor_Info motor_info;
            motor_info.bus_voltage = 12;
            motor_info.id = i.id;
            motor_info.sensor_position = actual_position;
            motor_info.sensor_velocity = curr_velocity;
            motor_info_map[motor_info.id] = motor_info;
        }
        else if (i.control_mode == ck_ros_base_msgs_node::Motor::POSITION)
        {
            float last_position = 0;
            if(motor_info_map.find(i.id) != motor_info_map.end())
            {
                last_position = motor_info_map[i.id].sensor_position;
            }
            ck_ros_base_msgs_node::Motor_Info motor_info;
            motor_info.bus_voltage = 12;
            motor_info.id = i.id;
            motor_info.sensor_position = (i.output_value + (last_position * 5.0)) / 6.0;
            motor_info_map[motor_info.id] = motor_info;
        }
        else if (i.control_mode == ck_ros_base_msgs_node::Motor::VELOCITY)
        {
            static ros::Time last_time = ros::Time::now();
            float delta_t = ros::Time::now().toSec() - last_time.toSec();
            float last_position = 0;
            if(motor_info_map.find(i.id) != motor_info_map.end())
            {
                last_position = motor_info_map[i.id].sensor_position;
            }
            ck_ros_base_msgs_node::Motor_Info motor_info;
            motor_info.bus_voltage = 12;
            motor_info.id = i.id;
            motor_info.sensor_position = last_position + (i.output_value * (delta_t / 60.0));
            motor_info.sensor_velocity = i.output_value;
            motor_info_map[motor_info.id] = motor_info;
        }
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "light_sim_ck_ros_base_msgs_node");

	sigintCalled = false;
	signal(SIGINT, sigint_handler);

	ros::NodeHandle n;
	node = &n;
    ros::Rate rate(RATE);

    load_config_params();

	ros::Subscriber motorConfig = node->subscribe("/MotorConfiguration", 100, motor_config_callback);
	ros::Subscriber motorControl = node->subscribe("/MotorControl", 100, motor_control_callback);
    ros::Subscriber sim_joystick = node->subscribe("/JoystickSimulation", 100, sim_joystick_subscriber);
    ros::Subscriber swerve_diag = node->subscribe("/SwerveDiagnostics", 100, swerve_diag_subscriber);
    ros::Subscriber sim_state = node->subscribe("/RobotSimulation", 100, sim_robot_state_subscriber);
    ros::Subscriber solenoid_control_subscriber = node->subscribe("/SolenoidControl", 100, solenoid_control_callback);

    while( ros::ok() )
    {
        ros::spinOnce();
        publish_robot_status();
        drive_motor_simulation();
        publish_motor_status();
        publish_imu_data();
        publish_joystick_status();
        rate.sleep();
    }


	return 0;
}

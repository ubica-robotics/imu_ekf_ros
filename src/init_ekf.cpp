#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "imu_ekf_ros/srv/init_request.hpp"
#include "math.h"
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ubica_rclcpp_utils/params.hpp>

// debugging
#include <iostream>

// data collection parameters
int num_data = 0;
int imu_counter = 0;

// data storage elements
Eigen::Vector3d sum_accel;
Eigen::Vector3d sum_gyro;

// public node handle pointer 
rclcpp::Node::SharedPtr n_ptr;
rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_ptr;
rclcpp::Service<imu_ekf_ros::srv::InitRequest>::SharedPtr service;

void handle_init_ekf(const imu_ekf_ros::srv::InitRequest::Request::SharedPtr req, 
                     const imu_ekf_ros::srv::InitRequest::Response::SharedPtr res)
{                     
        (void)req;
	// compute initial orientation
	Eigen::Vector3d g_b = sum_accel / num_data;

	// initial roll (phi) and pitch (theta)
	double phi = atan2(-g_b[1],-g_b[2]);
	double theta = atan2(g_b[0], sqrt(g_b[1]*g_b[1] + g_b[2]*g_b[2]));

	// set initial yaw to zero
	double psi = 0;

	// q is navigation to body transformation: R_bi
	// YPR: R_ib = R(yaw)R(pitch)R(Roll)
	// RPY: R_bi = R(-Roll)R(-Pitch)R(-yaw)	
	Eigen::Quaternion<double> q = Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX());
    q = q.inverse(); 

	// compute gyroscope biases
	Eigen::Vector3d gyro_biases = sum_gyro / num_data;

	// store in response
	res.get()->gyro_bias[0].data = gyro_biases[0];
	res.get()->gyro_bias[1].data = gyro_biases[1];
	res.get()->gyro_bias[2].data = gyro_biases[2];
	res.get()->init_orientation.x = q.x();
	res.get()->init_orientation.y = q.y();
	res.get()->init_orientation.z = q.z();
	res.get()->init_orientation.w = q.w();

	RCLCPP_INFO(n_ptr->get_logger(), "init_ekf: processed response");

	if (true)
	{
	    std::stringstream ss;
	    ss << gyro_biases << std::endl;
	    ss << q << std::endl;
	    std::string s = ss.str();
	    std::cout << s << std::endl;
	}
}

void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
	if (imu_counter < num_data)
	{
		// get accelerometer data
		geometry_msgs::msg::Vector3 a = msg->linear_acceleration;

		// get gyroscope data
		geometry_msgs::msg::Vector3 w = msg->angular_velocity;

		// add to matrix
		sum_accel -= Eigen::Vector3d(a.x,a.y,a.z);
		sum_gyro += Eigen::Vector3d(w.x,w.y,w.z);

		// increment counter
		imu_counter++;

	} else 
	{
		// stop receiving new imu data
		sub_imu_ptr = nullptr;

		// create the service
		service = n_ptr->create_service<imu_ekf_ros::srv::InitRequest>("/initialize_ekf", handle_init_ekf);

	}
}


// service handle

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	// create node handle and pointer
	n_ptr = rclcpp::Node::make_shared("init_imu_ekf_node");
	
	RCLCPP_INFO(n_ptr->get_logger(), "init_ekf node started.");

	// get number of data items to average from parameter server
	num_data = ubica_rclcpp_utils::declare_and_get_param(n_ptr, "num_data", 500);

	// imu callback
	sub_imu_ptr = n_ptr->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, imu_callback);

	while (rclcpp::ok()){
	    rclcpp::spin(n_ptr);
	}

	return 0;
}

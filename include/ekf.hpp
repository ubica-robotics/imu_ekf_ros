#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <chrono>
#include <string.h>
#include <iostream>

#include "imu_ekf_ros/srv/init_request.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ubica_rclcpp_utils/params.hpp>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>

namespace EKF
{
	void debug(std::string str)
	{
		std::cout << str << std::endl;
	}
}

// timer class to time parts of
class Timer 
{
public:
	Timer()
	{
		last = std::chrono::steady_clock::now();
	}
	void PrintDt(std::string label)
	{
		std::cout << label << ": " << CalculateTimeDiff() << std::endl;
	}

private:
	std::chrono::steady_clock::time_point last;
	float CalculateTimeDiff()
	{
		const auto old = last;
		last = std::chrono::steady_clock::now();
		const  std::chrono::duration<float> frame_time = last-old;
		return frame_time.count();
	}
};

// struct which contains constant parameters used in algorithm
struct EKF_struct {
		// global variables in filter
		Eigen::Matrix<double,12,12> Q = Eigen::Matrix<double,12,12>::Zero(); // noise matrix for IMU
		Eigen::Matrix<double,3,3> Ra = Eigen::Matrix<double,3,3>::Zero(); // noise matrix for accelerometer
		double g; // gravity
		double dt; // time step
		double num_data; // number of data collected for initialization
		Timer timer;
};

class IMU_EKF
{
public:

	IMU_EKF(rclcpp::Node::SharedPtr n);
	void initialize_ekf();

	Eigen::Matrix<double,10, 1> get_state() const
	{
		return m_state;
	}

private:

	// State vector [orientation, gyro bias, accel bias]
	Eigen::Matrix<double,10, 1> m_state; // state
	Eigen::Matrix<double, 9, 9> m_cov; // covariance
	Eigen::Matrix<double,3,1> m_g_pred; // predicted gravity
	Eigen::Matrix<double,3,1> m_g_pred_sum; // sum of all predicted gravities
	Eigen::Matrix<double,3,1> m_g_true; // true gravity vector
	Eigen::Matrix<double,3,3> Re; // measurement noise matrix
	EKF_struct m_filter; // filter object
	
	rclcpp::Node::SharedPtr n;

	// ROS related subscribers
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_subscriber;
	rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr m_sun_sensor_subscriber;
	rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr m_orientation_pub;

	// PI
	static constexpr double PI = 2*acos(0.0);

	// number of consecutive accelerometer measurments to declare robot is stationary
	const int NUM_STATIONARY = 125;

	// acceleration threshold to detect if robot is stationary
	static constexpr double ACCEL_THRESH = 0.1; // m/s^2

	// is the rover stationary
	bool m_rover_stationary = false; 

	// stationary counts
	int m_accel_counter = 0;

	// noise parameters for random walk
	const double m_lambda_g = -1.0/100;
	const double m_lambda_a = -1.0/100;

	enum SENSOR_TYPE
	{
		SUN_SENSOR = 1,
		ACCELEROMETER = 2
	};
	
	void sun_sensor_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);

	void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

	// F matrix. dx_dot = F*dx + G*w 
	void computeF(const Eigen::Matrix<double,3,1> &f_i, 
		const Eigen::Matrix<double,3,3> &R_body_to_nav_next, 
		Eigen::Matrix<double,9,9> &F) const;

	// G matrix. dx_dot = F*dx + G*w 
	void computeG(const Eigen::Matrix<double,3,3> &R_body_to_nav_next, 
		Eigen::Matrix<double,9,12> &G) const;

	// discrete state transition matrix and noise matrix
	void computePhiAndQdk(const Eigen::Matrix<double,3,1> &f_i, 
		const Eigen::Matrix<double,3,3> &R_body_to_nav_next, 
		Eigen::Matrix<double,9,9> &Phi, 
		Eigen::Matrix<double,9,9> &Qdk) const;

	// measurement update using gravity prediction
	void stationaryMeasurementUpdate(const Eigen::Matrix<double,3,3> & R_body_to_nav);

	// general Kalman filter
	void EKF(const Eigen::MatrixXd & H, 
		const Eigen::MatrixXd & R, 
		const Eigen::MatrixXd & z, 
		const SENSOR_TYPE sensor_type);

	// convert vector to 3x3 skew symmetric matrix
	void to_skew(const Eigen::Matrix<double,3,1> &v, Eigen::Matrix<double,3,3> &m) const
	{
		m << 0, -v(2), v(1),
		v(2), 0, -v(0),
		-v(1), v(0), 0;
	}
};

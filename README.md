# imu_ekf_ros
ROS2 Galactic version of a C++ package that fuses the accelerometer and gyroscope of an IMU to estimate attitude.  
ros2 launch cpp_ekf.launch for the C++ version.

-This node subscribes to sensor_msgs/IMU messages on the topic '/imu/data', containing accelerometer and gyroscope data.  

-This node publishes a quaternion to the topic '/quat'.  

Wait for 5 seconds for the initialization procedure. Update the following noise terms inside code for your IMU:

		sigma_xg # Gyro (rate) random walk  
		sigma_nug # Gyro white noise  
		sigma_xa # Accel (rate) random walk   
		sigma_nua # Accel white noise  

Tested with Xsens MTI-20 and Sensonor STIM300.
Primary reference is 'Aided Navigation: GPS with High Rate Sensors' by Jay A. Farrell, chapter 10.

# This message holds data from a 9-DOF IMU: orientation, angular velocity, and linear acceleration.

# Standard ROS message header, including timestamp and coordinate frame
std_msgs/Header header

# 3D orientation in quaternion form
geometry_msgs/Quaternion orientation
# Row-major representation of 3×3 covariance matrix
# (for a quaternion, not Euler angles). If unknown, set to all -1.
float64[9] orientation_covariance

# Angular velocity around each axis (rad/s)
geometry_msgs/Vector3 angular_velocity
# 3×3 covariance matrix (rad/s)^2
# If unknown, set to all -1.
float64[9] angular_velocity_covariance

# Linear acceleration along each axis (m/s^2)
geometry_msgs/Vector3 linear_acceleration
# 3×3 covariance matrix (m/s^2)^2
# If unknown, set to all -1.
float64[9] linear_acceleration_covariance


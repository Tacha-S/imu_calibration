# IMU calibration

The ROS package can calibrate a IMU sensor.

## Magnetic calibration

Bias correction is required when measuring geomagnetic field with magnetic sensors.
In magnetic calibration, the robot is rotated and the bias is estimated from the magnetic field data.

### Parameters

- `duration`: Duration of correction sensor data (default: `60.0[s]`)
- `velocity`: Angular velocity of rotation (default: `0.2[rad/s]`)
- `output_file`: Output file name (default: `"path/to/imu_calibration/config/mag.yaml"`)
- `visualize`: Visualize the calibration result (default: `false`)

### Inputs

- `mag` ([sensor_msgs/MagneticField](http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html))  
    Sensor value of the magnetic field to be calibrated

### Outputs

- `cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))  
    Command to rotate the robot

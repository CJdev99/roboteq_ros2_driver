# roboteq_diff_driver
[original work for ROS1](https://github.com/ecostech/roboteq_diff_driver)

ROS driver for the Roboteq SDC21xx, HDC24xx family of motor controllers in a differential-drive configuration.
Initially developed for SDC21xx and HDC24xx, but could work with other roboteq dual-channel motor drivers.

Subscribes to cmd_vel, publishes to odom


Does not require any MicroBasic script to operate.

## Usage

Clone to src directory of ros2 workspace, then 'colcon build'

Requires serial package. If not already installed:

Get the code:
    
    git clone https://github.com/wjwwood/serial.git

Build:

    make

Install:

    make install
    
    
Sample launch files in roboteq_diff_driver/launch.

## Motor Power Connections

This driver assumes right motor is connected to channel 1 (M1) of motor controller, and left motor is connected to channel 2 (M2). It also assumes a positive speed command will result in forward motion of each motor. Best to test motor directions using the roboteq utility software.


## TODO

- [X] Finish initial development of motor commands and odometry stream in ROS2
- [ ] Implement transform broadcasting with tf2
- [ ] Add roboteq/voltage, roboteq/current, roboteq/energy, and roboteq/temperature publishers
- [ ] Make topic names and frames configuration parameters configurable at runtime.
- [ ] Make robot configuration parameters configurable at runtime.
- [ ] Make motor controller device configuration parameters configurable at runtime.
- [ ] Make miscellaneous motor controller configuration parameters configurable at runtime.
- [ ] Implement dynamically enabled forced-run mode to verify correct motor power connections.
- [ ] Implement dynamically enabled self-test mode to verify correct motor power and encoder connections and configuration.

## Authors

* **Chad Attermann** - *Initial work* - [Ecos Technologies](https://github.com/ecostech)
* **Chase Devitt** - *initial ROS2 Migration*

# 🤖 CMPUT 412: Exercise 2 - Ros Development and Kinematics 🤖

Implementation of different components of ROS, and the basic application of differential drive kinematics and odometry. We only used 2 of the nodes that we created (led_controller_node.py and movement.py). The other ones are left for historical purposes.

## Run on your Duckiebot
1. Run the LED demo to get the led_pattern service started:
  * ```shell 
    dts duckiebot demo --demo_name led_emitter_node --duckiebot_name $BOT --package_name led_emitter --image duckietown/dt-core:daffy-arm64v8
    ```   
2. Build it:
  * ```shell 
    dts devel build -f -H MY_ROBOT.local
    ```   
3. Run it:
  * ```shell 
    dts devel run -H MY_ROBOT.local
    ```   

## References 🫡
- [LED node](https://github.com/anna-ssi/duckiebot/blob/50d0b24eab13eb32d92fa83273a05564ca4dd8ef/assignment2/src/led_node.py)
- [Wheel Odometry](https://github.com/anna-ssi/duckiebot/blob/50d0b24eab13eb32d92fa83273a05564ca4dd8ef/assignment2/src/wheel_odometry.py)
- [Robot Kinematics]( https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf)
- [Bag format](https://codeberg.org/akemi/duckietown/src/commit/70507322806ae0ff4e39fcbfa4bada3a7328a179/lab2/heartbeat-ros/packages/odometry_node/src/odometry_publisher_node.py)

## Made using: [this template](https://github.com/duckietown/template-basic)

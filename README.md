This repostory contain a package for mpu_6050 IMU and raspberry pi working with ROS2.

you need to install the following packages

sudo pip install adafruit_mpu6050 board  


The connection between mpu6050 and raspberry pi is shown in the image below. 

pi ---- mpu6050
v3 ---> vn
GND --> GND
SCL --> SCL
SDA --> SDA

after cloning the repo run 

colcon build --packages-select mpu_6050

in ros_ws

to run the node 

ros2 run mpu_6050 mpu_6050_node.py


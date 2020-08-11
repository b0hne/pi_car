# ros2_pi_car for ros2 Foxy Fitzroy
A simple node for a raspberry pi 3 controlled rc car using a PCA9685 and DC Motors

# install:
- pip3 install adafruit-circuitpython-pca9685
- pip3 install adafruit-circuitpython-servokit
- pip3 install board, busio


# features
- remote controlled via Twist Messages
- publishes live camera stream

# lauch
- $ ros2 launch pi_car pi_car.launch.py

# main.cpp
- implements same behaviour using in ROS Melodic

![Alt text](pi_car.png?raw=true "pi_car")

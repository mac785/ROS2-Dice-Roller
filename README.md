# ROS2-Dicebot
## Team 12: [Tech-no Logic](https://youtu.be/D8K90hX4PrE?si=nYKRTUP4AQPTekIG)

[Google Slides Presentation](https://docs.google.com/presentation/d/17dWJg9Xzjme5_viVJAJSdn36DbBFt2w16UpQ6tVTV7M/edit?usp=sharing)

[Youtube Demo Video](https://youtu.be/4z1GpTMS2zc?si=LoGROI-xu_ae7x18)

## Project Overview

### Package Overview

This project brings together 4 different systems inside ROS2 and Gazebo to control a simulated 2-wheel car all inside a docker container. The program can be broken down into 5 sections:  

1. Camera and Computer Vision   
2. Joystick Control  
3. Sensor Interpretation and Command  
4. Gazebo  
5. Launch

Please see below for a simple explainer of each package by section

#### Camera and Computer Vision
`webcam_publisher`: Reads webcam and publishes raw camera feed
`dice_detector`: Uses a YOLO model to determine the value of all dice in the camera view, and then summs the opposite side (bottom side) values

#### Joystick Control
`dualsense_node`: interprets the data from the ROS2 built-in `joy_node` and simplifies/trims-down the data

#### Sensor Interpretation and Command
`trigger_node`: takes in sensor data and joystick inputs and publishes `/cmd_vel` to control the robot

#### Gazebo
`my_robot_description`: contains all the information to describe our robot for the simulator
`my_robot_bringup`: contains all the instructions for launching the gazebo simulation properly with our robot

#### Launch
`bringup`: This simplifies simplifies things by making the entire program launch with one terminal command

### RQT Graph

<img width="3229" height="303" alt="Screenshot from 2025-11-19 22-00-56" src="https://github.com/user-attachments/assets/4e4446ca-efd9-49b4-b4f4-0b0960583707" />

The above graph is labeled by group just as in the above section, with a color key below for clarification
1. Camera and Computer Vision (Red)  
2. Joystick Control (Blue)  
3. Sensor Interpretation and Command (Green)  
4. Gazebo (Orange)  

## Preparing the Docker Container

Install Docker if you haven't already  
Ubuntu installation steps from the offial [Docker Installation Guide](https://docs.docker.com/engine/install/ubuntu/)

<img width="3840" height="2160" alt="Docker logo" src="https://external-content.duckduckgo.com/iu/?u=https%3A%2F%2Flogos-world.net%2Fwp-content%2Fuploads%2F2021%2F02%2FDocker-Symbol.png&f=1&nofb=1&ipt=4d20c27695c140709c95310391831ef7f120c0fc9df15b7eb7ca60749a8a55ee" />

Download this package:  
```
git clone https://github.com/mac785/ROS2-Dice-Roller
```

Perform all following commands from inside the cloned repository

Step 1: Build the image:  
```
docker build -t yolo_ros .
```

Once the image is build, launch the container:  
```
docker compose up -d
```  
(Note: you may need to adjust the contents of docker-compose.yml to accurately map camera and controller functionality)

Once the container is launched, open a terminal with:  
```
docker compose exec ros bash
```

## Running the Program

Once inside the container, you can run the full project with:  
```
ros2 launch bringup full_system.launch.py
```

Or, if you'd like to run each node individually:  

```
ros2 run webcam_publisher webcam_pub
```  
```
ros2 run dice_detector dice_node
```  
```
ros2 run joy joy_node
```  
```
ros2 run dualsense_node dualsense_node
```  
```
ros2 run trigger_node trigger_node
```  
```
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```


Once all nodes are online, roll some dice in front of the camera. Then, press directional buttons on the controller (D-Pad). This will move the robot.  
| Direction | Action       |
|-----------|-------------|
| Up        | Forwards    |
| Down      | Backwards  |
| Left      | Turn left  |
| Right     | Turn right |


The velocity of the robot is determined by the sum of the bottom faces of the dice (opposite of what's shown in the camera view). Roll a bunch of dice and see what happens!  

## References

`Dockerfile` based on and `requirements.txt` sourced from [yolo_ros by mgonzs13](https://github.com/mgonzs13/yolo_ros)

Image dataset [Dice by Roboflow user Workspace (workspace-spezm)](https://universe.roboflow.com/workspace-spezm/dice-0sexk)  

## Debugging

If your camera doesn't properly map, check your camera devices by using v4l-utils to find your camera info using the following command:  
```
v4l2-ctl --list-devices
```  
Then, update line 8 of `docker-compose.yml` so that it maps your camera of choice to /dev/usb_cam

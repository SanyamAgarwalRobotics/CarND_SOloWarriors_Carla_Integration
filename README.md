This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

[//]: # (Image References)
[image0]: ./imgs/system_arch.png "System Architecture Diagram"
[image1]: ./readme_images/labelImg_real_img.jpg "Label Real Image"
[image2]: ./readme_images/labelImg_sim_img.jpg "Label Sim Image"
[image3]: ./readme_images/labelImg_xml_real_img.jpg "XML Real Image"
[image4]: ./readme_images/traffic_node.png "Traffic Node"
[image5]: ./readme_images/waypoint_updater.png "Waypoint Updater Node"
[image6]: ./readme_images/dbw_node.png "DBW Node"
[image7]: ./readme_images/autoware_node.png "Autoware Node"

### Team: Sole Survivors

| Name                       | Email                       |
| -----------------------    | --------------------------- |
| Sanyam Agarwal (Team Lead) | aggarwal.sanyam@gmail.com   |
| Rakesh Paul                | rkpaul21@gmail.com          |
| Mainsh Pandey              | Manishpandey85@gmail.com    |

### 1. Submission checklist and requirements

- Launch correctly using the launch files provided in the capstone repo. The launch/styx.launch and launch/site.launch are used to test code in the simulator and on the vehicle respectively. The submission size limit is within 2GB.  
- Car smoothly follows waypoints in the simulator.  
- Car follows the target top speed set for the waypoints' twist.twist.linear.x in waypoint_loader.py. This has been tested by setting different velocity parameter(Kph) in /ros/src/waypoint_loader/launch/waypoint_loader.launch.  
- Car stops at traffic lights when needed.  
- Depending on the state of /vehicle/dbw_enabled, Car stop and restart PID controllers  
- Publish throttle, steering, and brake commands at 50hz.  
- For Simulated Track 1 we tested with velocity 40 Kmph and for track 2, 10 Kmph as the default velocity given in corresponding launch file.

### 2. System Architecture Diagram
Following diagram describes the overall system archicture showing ROS nodes and topics those are used to communicate among different parts of this 'Self Driving' vehicle susbsystems for this project
![alt text][image0]

#### 3. Overview
The starter repo has provided the skeleton of this architecture and all the ROS node has been provided with starter file. There are three ROS nodes in this diagram that we need to work on.

They are:

- Waypoint Updater Node
- DBW Node
- Traffic Light Detection Node

#### Waypoint updater

We update only the [waypoint_updater.py](https://github.com/SanyamAgarwalRobotics/CarND_SOloWarriors_Carla_Integration/blob/master/ros/src/waypoint_updater/waypoint_updater.py)

Waypoint updater node will publish waypoints from the from the car's current position to some waypoints ahead.
It will publish to `final_waypoints`.

Waypoint updater node subscribe to:
- `/current_pose` This contains current position of the car.
- `/base_waypoints` This contains the planned route of the car.
- `/current_velocity` This contains current velocity of the car.
- `/traffic_waypoints` This contains information about whether there is a red light ahead or there is no red light closeby.

Apart from those already mentioned in the udacity course.
Two functions are where we spend most of time developing on.
###### pose_cb
This is the callback function when we receive the current position of the car.
Whenever we receive a new position of the ego-car, we will check the corresponding waypoints of it.
If it is different from the previous one, we will publish the new waypoints.
If it is the same, for performance purpose, we avoid sending duplicate data.
This has an excpetion where if car is trying to resuming from stopped state, we will publish updated waypoints even when our position stays the same.

###### traffic_cb
This is the callback function when we receive the closest red light wayppints.
If the value is -1 meaning no red light nearby, we will publish the received `/base_waypoints`.
If there is a visible red light ahead of us, we would need to bring the car to stop and resuming when the light turns green.

We achieve both **stopping** and **resuming** by setting waypoint velocity.
For **stopping** we gradually slow the car down by setting the velocity from current position to stop line linearly decreasing to zero.
To make sure it does not overshoot we set extra_brake_wps so that if the car overshoot by accident it will still try to stop.

For **resuming** we gradually bring the car back to the pre-configured target velocity set by the launch file. We will set the waypoint velocity value to it linearly increasing it from 0 to target velocity.

Some flags like `self.stopping` and `self.resuming` are present to carry the current state of the car.


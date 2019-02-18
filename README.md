This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

[//]: # (Image References)
[image0]: ./imgs/system_arch.png "System Architecture Diagram"
[image1]: ./imgs/labelImg_real_img.jpg "Label Real Image"
[image2]: ./imgs/labelImg_sim_img.jpg "Label Sim Image"
[image3]: ./imgs/labelImg_xml_real_img.jpg "XML Real Image"
[image4]: ./imgs/traffic_node.png "Traffic Node"
[image5]: ./imgs/waypoint_updater.png "Waypoint Updater Node"
[image6]: ./imgs/dbw_node.png "DBW Node"
[image7]: ./imgs/autoware_node.png "Autoware Node"

### Team: Sole Warriors

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

#### DBW (Drive-By-Wire)

We update [dbw_node.py](https://github.com/SanyamAgarwalRobotics/CarND_SOloWarriors_Carla_Integration/blob/master/ros/src/twist_controller/dbw_node.py) and [twist_controller.py](https://github.com/SanyamAgarwalRobotics/CarND_SOloWarriors_Carla_Integration/blob/master/ros/src/twist_controller/twist_controller.py).

###### dbw_node.py
DBW node took in target twist command and publish the driving command: throttle, brake and steer.

This is the main class of the DBW node. It subscribe to:
- `/vehicle/dbw_enabled` whether DBW is enabled
- `/current_velocity` current velocity
- `/twist_cmd` target twist command including linear and angular velocity

It publishes to `/vehicle/steering_cmd` `/vehicle/throttle_cmd` `/vehicle/brake_cmd`.

Not much value added in this file by us here except we instantiate a controller that took in the subscribed information and outputs steering, throttle and break. 

The hardwork is in the twist_controller.py that contains the controller code.

###### twist_controller.py
Here we instantiates four extra controllers.
`accel_controller` is a PID controller to estimate the target acceleration.
`lowpass_filter` is a Low Pass Filter to smooth out the acceleration calcuated by the `accel_controller`.
`yaw_controller` is a Yaw Controller to calculate the target steering based on target and current twist command.
`throttle_controller` is another PID controler that took in acceleration output from the `lowpass_filter` and estimate the actual throttle needed.
We reset the `throttle_controller` when acceleration is not positive. 
If acceleration is negative, we calculate brake based on vehicle status specified by input controller.

We use the given simple pid controller(pid.py) was use to smoothly drive the car in both track, autonomously.

Since a safety driver may take control of the car during testing, we can not assume that the car is always following the PID commands. If a safety driver does take over, the PID controller will mistakenly accumulate error. For this reason dbw_node.py is subscribed to ros topic /vehicle/dbw_enabled and when this is off, it resets the PID controller's error function to zero.

P Component  

P controller steers the car proportion(inversely, Tau) to Cross Track Error. In this project where the car is driven by the waypoints, CTE was simulated as  

error = Desired velocity - current velocity of the car at a time  

If this number is negative means we want to reduce this error.  

In other word if the above is positive, car drives faster and if negative, car slows down. With a higher Tau the car will oscillate faster.

We used Kp as 5.

D Component  

In PD-controller, for this prooject, when the car is reducing the speed to reduce the error, it won’t just go shooting for the reference velocity but it will notice that it is already reducing the error and as the error is becoming smaller overtime, it counter steers i.e it steers up again, this will allow it to gracefully approach the reference velocity

We do not use the D controller  here so Kd was set to zero.

I Component  

A car might have a form of wheels not aligned appropriately, in robotics it is called Systematic bias. Systematic bias will significantly increase the CTE in PD controller, so the differential term will not be able to compensate for this. This is where the I components come to play which is measured by the integral or the sum of the CTEs over time. Integral coefficient (Ki) should be carefully optimized in small steps as it has a large impact on the overall performance. We use a small value for ki = 0.1 based on trail and error  

* Low pass filter  
An average of current velocities are used to derive the current velocity in order to avoid any sporadic spike.  

* Throttle: The PID controller in the twist_controller.py calculates the throttles, which is calculated as Kp*vel_err + Ki * vel_err_integral + Kd* vel_err_delta. The vel_err is calculated as the difference between the target_vel and the current_vel. The PID controller is only active if the dbw is enabled to avoid error accumulation. The Kp, Ki, Kd provided by Udacity works well in the simulator. Some other value were experimented and no significant performance improve have been seen. For the car in real world further fine tuning might be necessary.

* Steering: For getting the steering value we use the Yaw_controller.py provided to us. It adjusts the angular velocity setpoint, based on our current speed. It uses the formula linear velocity = angular velocity * radius in physics.

* Brake: Torque value for braking = deceleration x vehicle mass x wheel radius

#### Traffic Light Detection

We got the images of traffic light captured by simulator's camera and that by Carla's(Udacity's self driving car).We placed simulator and real car's images under `simulator_data` and `realimg_data` folder respectively. Further divided each of them into `train` and `test` folders with 30% images in `test` folder. 

We used [labelImg](https://github.com/tzutalin/labelImg) to draw bounding box around the traffic light object(s) in the images and label them as either `red` or `yellow` or `green` or `unknown`

Labelling `real` images:

![alt text][image1]

Labelling `sim` images:

![alt text][image2]

labelImg creates `.xml` for each image with information of the image itself and details of the bounding box and its label:

![alt text][image3]


Using a helper function [`xml_to_csv.py`](https://github.com/datitran/raccoon_dataset/blob/master/xml_to_csv.py), all the `.xml` are merged into a single `.csv` file. Another helper function [`generate_tfrecord.py`](https://github.com/datitran/raccoon_dataset/blob/master/generate_tfrecord.py) is used to generate the TFRecord `.record` file which is the file format needed by TensorFlow. Slight modifications were made to the above two helper functions and are placed in this repository.

##### Model Selection

We setup for the F-RCNN model due to its remarkable speed and accuracy

##### Training and exporting for inference

All the required steps for training and exporting the inference graph are describe details  [here]()


#### 2.1.1.b. Classification

tl_detector.py is subscribed for topics /image_color, which provides an image stream from the car's camera. These images are used to determine the color of upcoming traffic lights. Based on Camera encoding this images is converted into RGB i.e. (cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)) before feed into for classification.  

The classification module, tl_classifier.py is based on [CarND Object Detection Lab](https://github.com/udacity/CarND-Object-Detection-Lab)  A new ros parameter named 'scenario' has introduced in tl_detector/launch/tl_detector.launch and in tl_detector/launch/tl_detector_site.launch to locate the frozen inference graph at runtime. By default FRCNN is set for Real track i.e. tl_detector/light_classification/frozen_models/faster_rcnn_resnet101_coco_2018_01_28/frozen_inference_graph.pb


### 3. References

[TensorFlow Models](https://github.com/tensorflow/models)  
[The PASCAL Visual Object Classes Homepage](http://host.robots.ox.ac.uk/pascal/VOC/)  
[udacity/CarND-Object-Detection-Lab](https://github.com/udacity/CarND-Object-Detection-Lab)   
[Alex_Lechner](https://github.com/alex-lechner/Traffic-Light-Classification)

## Installation

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images


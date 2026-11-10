# CSE_180_Project
CSE180Project


## Details: ##

FINAL PROJECT: Search the warehouse

COLLABORATION POLICY: this project can be solved in teams of at most three students.

GENERATIVE AI POLICY: you can use AI, both as a coding assistant and, as described below, as part of your robot’s software.

The project will be presented in class during the last week of classes (during your assigned lab session). More details are forthcoming.

This time you will use a different launch file:

ros2 launch gazeboenvs tb4_warehouse.launch.py
It starts the same robot in new environment (warehouse), plus the whole Nav2 stack. This includes a map of the warehouse, AMCL localization, planners and controllers. You no longer have to write the low-level steering yourself. Nav2 drives the robot to any pose on the map and avoids obstacles along the way.

Your task is to search the warehouse for people. The robot must explore the warehouse and recognize people with its camera. For each person it finds, it reports the person’s position on the map. Nobody will tell you how many people there are or where they stand. To recognize them you will use a pretrained neural network for object detection. This is the “AI” part of the project: the network turns pixels into the label “person” plus a box in the image. Turning that box into a point on the map is your job.

The sensors and frames

map: the frame of the warehouse map. AMCL localizes the robot in it. In this simulation, map coincides with the Gazebo world frame, so you can check your results against object positions in Gazebo.
The robot frame and odom: as in previous labs.
The camera optical frame: the robot carries an RGB-D camera. Its images are expressed in an optical frame, which is oriented differently from every frame you have used so far.
What you need to do

In your CSE180 workspace, create a package called search. If you write your detector in Python (see step 5), create a second package for it, called detector.
Start the simulation with the command above. To watch the map, the robot’s localization and your markers, start RViz as well, either with use_rviz:=True or by launching it separately.
Explore the system before writing any code.
Which topics carry the color image, the depth image and the camera calibration? What are their message types? What are the image resolution, the encoding of each image, and the frame they are expressed in?
Inspect camera_info. Which entries are fx, fy, cx and cy? Using the pinhole model from class, write down how a pixel (u, v) with depth d becomes a 3D point in the camera frame.
Use tf2_echo between the robot frame and the camera optical frame. Which way do the x, y and z axes of the optical frame point? Why is it not oriented like the robot frame?
What unit does the depth image use? What value does a pixel hold when there is nothing within range?
Which action servers does Nav2 offer (ros2 action list)? Read the MRTP navigation package. Which Navigator methods will you need?
Find the two people already in the world (check the Gazebo entity tree). Write down their positions. You will use them to test your system.
Plan the search. Open the map in RViz and choose a sequence of waypoints from which the camera can see every open area of the warehouse. Remember the camera’s field of view (about 70°) and the range at which your detector still recognizes a person, which you will measure in step 5. At each waypoint the robot may turn in place, using Spin, to look around. Hardcode the waypoints in your code, and explain how you chose them in your report.
The detector node. Write a node that subscribes to the color image, runs a pretrained detector, and publishes the people it finds as a vision_msgs/msg/Detection2DArray. Each detection gives a bounding box and a confidence score. Copy the image’s header into the output message, so that later nodes know exactly when the image was taken.
You may use any pretrained detector, for example a YOLO model through the ultralytics Python package, or an ONNX model through OpenCV’s dnn module in C++. You may write this node in Python. It is the only part of the project where Python is allowed.
Keep only detections of class “person” with confidence above a threshold. Choose the threshold and justify it in your report.
Running the network on every frame may be too slow on your machine. If so, process at most a few frames per second.
Measure your detector: drive the robot with teleop toward a person, and record the distance at which detections become reliable. You will need it for step 4.
The search node (C++). It has:
a Navigator (from MRTP) to visit the waypoints in order and spin at each one;
subscriptions to the detections, the depth image and camera_info;
a tf2_ros::Buffer and TransformListener.
For every person detection:
Take the depth from the depth image inside the bounding box. Use a robust value, such as the median of the valid pixels in the central part of the box, not a single pixel. A single pixel may land on the wall behind the person.
Compute the 3D point in the camera optical frame with the pinhole model, using the parameters from camera_info.
Transform the point into map with TF2. Use the transform at the image’s timestamp: the robot may be turning while the image is taken.
Merge duplicates. The same person is seen many times, from many places. If a new detection falls within M=0.6 of a person already found (see below), treat it as the same person and update that person’s estimate (for example, with a running average). Otherwise, add a new person. Ignore people found from only a single detection: they are likely false positives.
While the search runs, publish a visualization_msgs/msg/MarkerArray with one marker per person found, so that the results can be watched live in RViz. When the last waypoint is done, the robot returns to its starting pose and prints the final list: for each person, its map coordinates and the number of detections it was based on.
To build your solution you will have to update CMakeLists.txt and package.xml (and setup.py for a Python package). Use use_sim_time:=true for all your nodes.
Checking your work

Run your system with the default world. It should find both people, and each reported position should be within about 0.5 m of the true one. Then add more people yourself, using Fuel person models from Gazebo’s resource spawner, including one partly hidden behind a shelf. Also place one person in an area your waypoints cover poorly, and see what happens.

We will grade your project with a world in which people are placed in locations you do not know. The number of people found, the accuracy of their positions, and the absence of false positives all count. So do the waypoints you planned in step 4: if they leave part of the warehouse unseen, the people standing there will be missed.

Submitting your work

Submit a zipped file containing your entire workspace, and a short report (at most 2 pages) that covers:

your answers to step 3;
how you chose the waypoints, with a picture of them on the map;
the detector you used, its threshold, and the range you measured;
a table comparing your estimates with the true positions in the default world;
one failure you observed and its cause.
You will demonstrate and discuss your code during your lab session in the last week of the course.

**A note on the use of generative AI**

This time AI is part of the robot itself, and you may also use AI tools to write code. The coordinate changes are still the heart of the project, and you will be asked to explain them at the demo:

why the optical frame is oriented the way it is;
why the transform at the image’s timestamp is needed;
what your merge rule does when two people stand close together.
 

## End ##
# Micropolis Robotics - Robotics Software Engineer Technical Assesment

This is task defined project to asses the new hirings for Robotics Software Engineer
It helps the company to evaluate the applicants knowledge and skills in the tools and frameworks used in the department.

## Areas Covered By This Test
- Implementation and coding skills
- C++ and Python profcincy
- Robot Operation Systems (ROS)
- Robotics Fundementals
- Autonomous Navigation Fundementals
- GUI development
- Software Integration

## Guide and Tips
- Fork the repo to your account, and reply to the email with your repo fork that contains the soloutions once you finish **(Any reply after two weeks of the email wil not be accepted!!!)**.</br>
- Try to utilize known/open-source tools as possible.</br>
- Edit the README.md file in your fork, and add the steps and exxplination of your solution for each milestone.

## Project Overview
You are given a ROS1 workspace that contains a ready to use setup for a car-like robot equibbed with a depth camera and a 3D LiDAR, placed in a virtual world within Gazebo Simulator.
The target goal for this project, is to develop a minimal user controlled autonomous navigation functionality for the robot provided.

## How To Run
- Add the models used in the world file:
~~~bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{Add your workspace path}/src/mybot_pkg/models
~~~

- Check and install the corresponding dependencies:

~~~bash
# Available in RSE_ws
./install.sh
~~~


- Launch the robot inside Gazebo:
~~~bash
# Launch the Robot inside Gazebo World
roslaunch mybot_pkg gazebo.launch
~~~

## Milestones
To achieve the desired goal, we break it down to smaller     to be achieved in order based on its debendency for the the next step.


### 1 - Preform a SLAM on the provided world
First, you need to map the robot world so it can understand it for later operations. </br>
Utilize your knowledge of SLAM algorithms to produce a digital map of the world.

### 2 - Offline Localization
Next, to move the robot autonomously around the map you need to localize the robot in real-time without using SLAM (offline localization).</br>
Implement/Use a localization algorithm to localize the robot in the map, and test that your localization is working by movibg the robot manyually arround the map and validate the localization output.

### 3 - Autonomous Navigation with Obstacle avoidance
Once you have a represntation of the environment and you can localize your robot within it, you can then start the autonomous navigation of the robot.</br>
Implement/Use an autonomous navigation algorithm to drive the robot by itself to a defined goal that can be set in the RViz GUI, while avoiding any obstacle.

### 4 - External GUI Teleoperation
To make sure a smother operation of the robot when deploying it in the field, it's better to have a user friendly remote operation capabilties.</br>
Develop a GUI that allows use to remotly control the robot from outside a ROS environment.
Feel free to pick whatever framework or stack you like (C++ Qt, Python, Web-based GUI, etc...).
**NOTE:** Implement only the basic functionality (Drive, Steer).

### 5 - User Defined Navigation (Open)
Develop the previous milestone and adopt it so the user of your GUI can also perform the Navigation functionality (Sendg Waypoints/Goal, Mapping, etc...).

### (Optional) - Develop an Odometry Source for the robot
The very first required components to start working on any autonomous functionality are the position, orientation, velocity feedback of the robot.</br>
If we ignore the Odometry feedback provided by Gazebo, based on the robot description provided and the sensor set, develop a node that produce an Odometry feedback as accurate as possible.



```bash
GOOD LUCK!
```

</br>

# The Implementation
### 1 - Preform a SLAM on the provided world
Due to the existence of many peace algorithms, I have tried several methods:

###### - RTAB-Map
~~~bash
<node pkg="rtabmap_ros" type="rtabmap" name="rtabmap" output="screen">
    <param name="rtabmap_args" value="delete_db_on_start" />
    <remap from="/camera/color/image_raw" to="/steer_bot/camera/color/image_raw" />
    <remap from="/camera/depth/image_raw" to="/steer_bot/camera/depth/image_raw" />
    <remap from="/odom" to="/steer_bot/ackermann_steering_controller/odom" />
</node>
~~~
###### - Octomap_server
~~~bash
<node pkg="octomap_server" type="octomap_server_node" name="octomap_server" output="screen">
    <param name="base_frame_id" type="string" value="base_link" />
    <param name="frame_id" type="string" value="odom" />
    <param name="point_cloud_topic" type="string" value="/steer_bot/points" />
    <remap from="cloud_in" to="/steer_bot/points" />
</node>
~~~
###### - Convert 3D to 2D using pointcloud_to_laserscan and apply gmapping 
~~~bash
<node pkg="pointcloud_to_laserscan" type="pointcloud_to_laserscan_node" name="pointcloud_to_laserscan">
    <remap from="cloud_in" to="/steer_bot/points" />
    <remap from="scan" to="/steer_bot/scan" />
    <rosparam>
        target_frame: base_link
        transform_tolerance: 0.01
        min_height: 0.0
        max_height: 1.0
        angle_min: -1.5708
        angle_max: 1.5708
        angle_increment: 0.0087
        scan_time: 0.3333
        range_min: 0.45
        range_max: 4.0
        use_inf: true
        inf_epsilon: 1.0
        concurrency_level: 1
    </rosparam>
</node>

<!-- Launch Gmapping -->
<node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
    <param name="base_frame" value="base_link" />
    <param name="odom_frame" value="odom" />
    <param name="map_frame" value="map" />
    <remap from="scan" to="/steer_bot/scan" />
    <param name="map_update_interval" value="5.0" />
</node>
~~~

<br/>

### 2 - Offline Localization
Using AMCL 

~~~bash
<!-- Launch the map server -->
<node name="map_server" pkg="map_server" type="map_server" args="$(find mybot_pkg)/maps/map.yaml" />

<!-- Launch the AMCL node -->
<node name="amcl" pkg="amcl" type="amcl">
    <param name="odom_model_type" value="diff" />
    <param name="base_frame_id" value="base_link" />
    <param name="odom_frame_id" value="odom" />
    <param name="global_frame_id" value="map" />
</node>
~~~

<br/>

### 3 - Autonomous Navigation with Obstacle avoidance
Using move_base package:
~~~bash
<node name="move_base" pkg="move_base" type="move_base" respawn="false" output="screen">
    <param file="$(find mybot_pkg)/config/my_robot_move_base_params.yaml" command="load"/>
</node>
~~~

<br/>

### 4 - External GUI Teleoperation
Using tkinter library
~~~bash
import tkinter as tk
from std_msgs.msg import Twist
import rospy

class TeleopGUI:
    def __init__(self):
        # Initialize ROS Node
        rospy.init_node('teleop_gui', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # Initialize GUI
        self.root = tk.Tk()
        self.root.title('Robot Teleoperation')
        # Create widgets
        self.forward_button = tk.Button(self.root, text='Forward', command=self.move_forward)
        self.backward_button = tk.Button(self.root, text='Backward', command=self.move_backward)
        self.left_button = tk.Button(self.root, text='Left', command=self.turn_left)
        self.right_button = tk.Button(self.root, text='Right', command=self.turn_right)
        # Layout widgets
        self.forward_button.grid(row=0, column=1)
        self.backward_button.grid(row=2, column=1)
        self.left_button.grid(row=1, column=0)
        self.right_button.grid(row=1, column=2)
        # Bind arrow keys for continuous control
        self.root.bind('<Up>', lambda event: self.move_forward())
        self.root.bind('<Down>', lambda event: self.move_backward())
        self.root.bind('<Left>', lambda event: self.turn_left())
        self.root.bind('<Right>', lambda event: self.turn_right())
        # Run the GUI
        self.root.mainloop()
    def send_command(self, linear, angular):
        twist_cmd = Twist()
        twist_cmd.linear.x = linear
        twist_cmd.angular.z = angular
        self.cmd_pub.publish(twist_cmd)
    def move_forward(self):
        self.send_command(0.2, 0.0)
    def move_backward(self):
        self.send_command(-0.2, 0.0)
    def turn_left(self):
        self.send_command(0.0, 0.2)
    def turn_right(self):
        self.send_command(0.0, -0.2)

if __name__ == '__main__':
    TeleopGUI()

~~~

And add the node in the launch file

~~~bash
<node name="teleop_gui_node" pkg="teleop_bot" type="teleop_gui.py" output="screen" />
~~~


### 5 - User Defined Navigation (Open)
Using tkinter library
~~~bash
import rospy
from std_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import Tkinter as tk
import actionlib

class TeleopGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Robot Teleoperation")

        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist_cmd = Twist()

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        self.create_widgets()

    def create_widgets(self):
        # Forward Button
        forward_btn = tk.Button(self.master, text="Forward", command=self.move_forward)
        forward_btn.grid(row=0, column=1)

        # Backward Button
        backward_btn = tk.Button(self.master, text="Backward", command=self.move_backward)
        backward_btn.grid(row=2, column=1)

        # Left Button
        left_btn = tk.Button(self.master, text="Left", command=self.move_left)
        left_btn.grid(row=1, column=0)

        # Right Button
        right_btn = tk.Button(self.master, text="Right", command=self.move_right)
        right_btn.grid(row=1, column=2)

        # Set Goal Button
        set_goal_btn = tk.Button(self.master, text="Set Goal", command=self.set_goal)
        set_goal_btn.grid(row=1, column=1)

    def move_forward(self):
        self.twist_cmd.linear.x = 0.2
        self.twist_cmd.angular.z = 0.0
        self.twist_pub.publish(self.twist_cmd)

    def move_backward(self):
        self.twist_cmd.linear.x = -0.2
        self.twist_cmd.angular.z = 0.0
        self.twist_pub.publish(self.twist_cmd)

    def move_left(self):
        self.twist_cmd.linear.x = 0.0
        self.twist_cmd.angular.z = 0.2
        self.twist_pub.publish(self.twist_cmd)

    def move_right(self):
        self.twist_cmd.linear.x = 0.0
        self.twist_cmd.angular.z = -0.2
        self.twist_pub.publish(self.twist_cmd)

    def set_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'  # Change if necessary
        goal.target_pose.pose.position.x = 2.0  # Update with desired coordinates
        goal.target_pose.pose.position.y = 2.0
        goal.target_pose.pose.orientation.w = 1.0

        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

        if self.move_base_client.get_state() == GoalStatus.SUCCEEDED:
            print("Goal reached successfully!")
        else:
            print("Failed to reach the goal.")

if __name__ == '__main__':
    rospy.init_node('teleop_gui_node', anonymous=True)
    root = tk.Tk()
    teleop_gui = TeleopGUI(root)
    root.mainloop()

~~~ 
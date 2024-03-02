#!/usr/bin/env python

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

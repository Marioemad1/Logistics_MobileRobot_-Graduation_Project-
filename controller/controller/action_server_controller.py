#!/usr/bin/env python3
import rclpy
import threading
from rclpy.node import Node
from rclpy.action import ActionServer , GoalResponse , CancelResponse
from rclpy.action.server import ServerGoalHandle 
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

class ControllerNode(Node): #create the class and inharet from node
    def __init__(self): 
        super().__init__("controller_server") #call the init from the Node class
        self.declare_parameter("initial_postion.x",0.0)
        self.declare_parameter("initial_postion.y",0.0)
        self.declare_parameter("initial_postion.theta",0.0)

        self.init_pose_x_= self.get_parameter("initial_postion.x").value
        self.init_pose_y_= self.get_parameter("initial_postion.y").value
        self.init_pose_z_= self.get_parameter("initial_postion.theta").value
        self.server_goal_handel_: ServerGoalHandle = None # pyright: ignore[reportAttributeAccessIssue]
        self.goal_lock_= threading.Lock()
        self.current_position_=  self.get_parameter("initial_postion").value
        self.robot_move_action_server_ = ActionServer(
            self,
            PoseStamped,
            "set_point",
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handel_acepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
             callback_group=ReentrantCallbackGroup())
        
        self.nav_= BasicNavigator()
        self.init_pose_= self.create_goal_stamp(
            self.nav_,self.init_pose_x_,self.init_pose_y_,self.init_pose_z_)

        self.get_logger().info("controller server is running...")


#************************************************************************************

    def goal_callback(self,goal:PoseStamped):
        
        self.get_logger().info("Accepting the Goal...")
        return GoalResponse.ACCEPT

#*****************************************************************************************

    def handel_acepted_callback(self , goal_handel:ServerGoalHandle):
        #after accepting the goal here in accept state choose to process to execute 
        #do process to the aceppted goals 
        self.get_logger().info("execute the goal...")
        goal_handel.execute()

#******************************************************************************************

    def cancel_callback(self, goal_handel:ServerGoalHandle):
        #choose if to accept the cancel request or not 
        self.get_logger().info("accept the cancel request")
        return CancelResponse.ACCEPT

#********************************************************************************************

    def execute_callback(self, goal_handel:ServerGoalHandle):
        #here we type the action that we will process on goals
        pass

    def create_goal_stamp( self, navigation : BasicNavigator, x,y,z):
        q_x,q_y,q_z,q_w = tf_transformations.quaternion_from_euler(0.0,0.0,z)
        
        #---------------send the robot position---------------------
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.header.stamp= navigation.get_clock().now().to_msg()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = 0.0
        target_pose.pose.orientation.x = q_x
        target_pose.pose.orientation.y = q_y
        target_pose.pose.orientation.w = q_z
        target_pose.pose.orientation.z = q_w
        
        return target_pose

def main(args=None):
    rclpy.init(args=args)   
    node=ControllerNode()   
    rclpy.spin(node, MultiThreadedExecutor())                    
    rclpy.shutdown()                   



#if the name main called in the terminal call the main function
if __name__ == "__main__":
    main()
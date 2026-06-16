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
from logistic_msg.action import MoveRobot
import tf_transformations
from rclpy.duration import Duration
from std_msgs.msg import Float64MultiArray

class ControllerNode(Node): #create the class and inharet from node
    def __init__(self): 
        super().__init__("logistic_action_server") #call the init from the Node class
        self.declare_parameter("initial_postion.x",0.0)
        self.declare_parameter("initial_postion.y",0.0)
        self.declare_parameter("initial_postion.theta",0.0)

        self.init_pose_x_= self.get_parameter("initial_postion.x").value
        self.init_pose_y_= self.get_parameter("initial_postion.y").value
        self.init_pose_z_= self.get_parameter("initial_postion.theta").value
        self.server_goal_handel_: ServerGoalHandle = None # pyright: ignore[reportAttributeAccessIssue]
        self.goal_lock_= threading.Lock()
        self.robot_move_action_server_ = ActionServer(
            self,
            MoveRobot,
            "start_action",
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handel_accepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())
        
        self.nav_= BasicNavigator()
        self.init_pose_= self.create_goal_stamp(
            self.nav_,self.init_pose_x_,self.init_pose_y_,self.init_pose_z_)
        
        
        
        self.home_point_= self.create_goal_stamp(self.nav_,0.0,0.0,0.0)
        self.pick_in_point_= self.create_goal_stamp(self.nav_,0.0,0.0,0.0)
        self.pick_off_point_=self.create_goal_stamp(self.nav_,0.0,0.0,0.0)


        #fix the massage type and the topic name based on the nema 
        #self.nema_state_sub_ = self.create_subscription(
        #    String,"topic_name",self.nema_state_sub_callback,10)
        
        #self.nema_state_msg = String()

        #self.nema_state_msg.data = None  
        
        self.nema_command_pub_ = self.create_publisher(Float64MultiArray,"/Nema_controller/commands",10)

        self.nav_.setInitialPose(self.init_pose_)
        self.get_logger().info("controller server is running...")


#************************************************************************************

    def goal_callback(self,goal:MoveRobot.Goal):

        if self.server_goal_handel_ is not None and self.server_goal_handel_.is_active:
            self.get_logger().info("there is a running goal, wait")
            return GoalResponse.REJECT
        
        self.get_logger().info("Accepting the Goal...")
        return GoalResponse.ACCEPT

#*****************************************************************************************

    def handel_accepted_callback(self , goal_handel:ServerGoalHandle):
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
        with self.goal_lock_:
            self.server_goal_handel_ = goal_handel

        result = MoveRobot.Result()

        nema_command = Float64MultiArray()
        nema_command.data = [0.0]
        if not goal_handel.is_active:
            #if the goal is abouretd by error go to the home point
            self.get_logger().error("returnning to home due to error")
            self.nav_.goToPose(self.home_point_)
            while not self.nav_.isTaskComplete():
                 feedback = self.nav_.getFeedback()
                 self.get_logger().warn(f"the feedback {feedback}")
                 time.sleep(0.1)
            result.pose_x = feedback.current_pose.pose.position.x # type: ignore
            result.pose_y = feedback.current_pose.pose.position.y  # type: ignore
            return result
        
        if goal_handel.is_cancel_requested:
            self.get_logger().warn("returnning to home due to cancel request")
            self.nav_.goToPose(self.home_point_)
            while not self.nav_.isTaskComplete():
                 feedback = self.nav_.getFeedback()
                 self.get_logger().warn(f"the feedback {feedback}")
                 time.sleep(0.1)
            result.pose_x = feedback.current_pose.pose.position.x # type: ignore
            result.pose_y = feedback.current_pose.pose.position.y  # type: ignore
            return result
        
        #first go to the pick_up_point
        self.get_logger().info("going to the pick up point")
        self.nav_.goToPose(self.pick_in_point_)
        while not self.nav_.isTaskComplete():
            nav_feedback = self.nav_.getFeedback()
            if nav_feedback:
                action_feedback = MoveRobot.Feedback()
                action_feedback.pose_x = nav_feedback.current_pose.pose.position.x
                action_feedback.pose_y = nav_feedback.current_pose.pose.position.y
                
                goal_handel.publish_feedback(action_feedback)
            time.sleep(0.1)

        self.get_logger().info("point reached")
        
        #after going to the pick up lets pick up 
        nema_command.data = [1.0]
        self.nema_command_pub_.publish(nema_command)
        time.sleep(24)

        #while(self.nema_state_msg.data != "1"):
        #    self.get_logger().info("waitting for the nema to left up")
        #    time.sleep(0.1)
        
        self.get_logger().info("the cargo is lifted")

        # wait just for one second for safty
        self.get_clock().sleep_for(Duration(seconds=1))

        self.get_logger().info("going to the pick off point")

        self.nav_.goToPose(self.pick_off_point_)
        while not self.nav_.isTaskComplete():
            nav_feedback = self.nav_.getFeedback()
            if nav_feedback:
                action_feedback = MoveRobot.Feedback()
                action_feedback.pose_x = nav_feedback.current_pose.pose.position.x
                action_feedback.pose_y = nav_feedback.current_pose.pose.position.y

                goal_handel.publish_feedback(action_feedback)

            time.sleep(0.1)

        self.get_logger().info("point reached")
        nema_command.data = [0.0]
        self.nema_command_pub_.publish(nema_command)
        self.get_clock().sleep_for(Duration(seconds=1))

        nema_command.data = [-1.0]
        self.nema_command_pub_.publish(nema_command)
        time.sleep(24)
        #while(self.nema_state_msg.data != "-1"):
        #    self.get_logger().info("waitting for the nema to go down")
        #    time.sleep(0.1)

        self.get_logger().info("the cargo is down")

        nema_command.data = [0.0]
        self.nema_command_pub_.publish(nema_command)
        self.get_clock().sleep_for(Duration(seconds=1))

        self.get_logger().info("going to the home")



        self.nav_.goToPose(self.home_point_)
        while not self.nav_.isTaskComplete():
            feedback = self.nav_.getFeedback()
            self.get_logger().info(f"the feedback {feedback}")


        self.get_logger().info("task is done")
        self.get_clock().sleep_for(Duration(seconds=1))

        goal_handel.succeed()

        feedback = self.nav_.getFeedback()
        if feedback:
            result.pose_x = feedback.current_pose.pose.position.x # type: ignore
            result.pose_y = feedback.current_pose.pose.position.y  # type: ignore
        return result

    
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
        target_pose.pose.orientation.w = q_w
        target_pose.pose.orientation.z = q_z
        
        return target_pose
    

    #def nema_state_sub_callback(self,msg:String):
    #    self.nema_state_msg.data = msg.data
    #    self.get_logger().info(f"[NEMA] {msg.data}")

def main(args=None):
    rclpy.init(args=args)   
    node=ControllerNode()   
    rclpy.spin(node, MultiThreadedExecutor())                    
    rclpy.shutdown()                   



#if the name main called in the terminal call the main function
if __name__ == "__main__":
    main()
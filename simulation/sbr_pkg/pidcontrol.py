'''
error = setpoint - actual_state
error is a function of time
pass error function into pid controller and sum outputs
turn this output into a tangeable action - pass into sigmoid or other 'squishification' function from -1 to 1
'''

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, Quaternion, Point
from std_srvs.srv import Empty
from std_msgs.msg import Int32 #sensor_msgs/msg/Imu
from sensor_msgs.msg import Imu
import math 
import numpy as np
from gazebo_msgs.srv import DeleteEntity, SpawnEntity, SetEntityState
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
import time

class PID_control(Node):
    def __init__(self):
        super().__init__("PID_Control")

        self.kp = 21
        self.ki = 140
        self.kd = 0.8
        self.setpoint = 0
        self.prev_error = 0
        self.callback_hz = 30
        self.error_integral = 0
        
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.client_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.timer = self.create_timer(1/self.callback_hz, self.correction, self.timer_cb_group)
        self.qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10
        )
        self.latest_msg = Imu()
        self.imu_sub = self.create_subscription(Imu, "imu/data", self.new_imu_data, self.qos_profile)
        self.client = self.create_client(SetEntityState, "/set_entity_state", callback_group=self.client_cb_group)
        self.pause = self.create_client(Empty, "/pause_physics", callback_group=self.client_cb_group)
        self.unpause = self.create_client(Empty, "/unpause_physics", callback_group=self.client_cb_group)
        self.steps = 0


    def new_imu_data(self, msg):
        self.latest_msg = msg

    def euler_from_quaternion(self,x,y,z,w):
        t0 = 2.0 * (w * x + y*z)
        t1 = 1.0 - 2.0 * (x*x + y*y)
        roll_x = math.atan2(t0,t1)
        t2 = 2.0 * (w*y - z*x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = 2.0 * (w*z +x*y)
        t4 = 1.0 - 2.0 *(y*y + z*z)
        yaw_z = math.atan2(t3,t4)

        return roll_x, pitch_y, yaw_z
    def get_observation(self):
        observation_data = self.latest_msg
        r, p, y = self.euler_from_quaternion(observation_data.orientation.x,observation_data.orientation.y,observation_data.orientation.z,observation_data.orientation.w)
        observation = {
            "roll" : r,
            "pitch" : p,
            "yaw" : y,
            "angular" : observation_data.angular_velocity.y
        }
        return observation
    

        

    def sigmoid(self, x):
        return 1.0/(1.0 + np.exp(-x))
    def make_action(self, action):
        vel_cmd = Twist()
        #self.get_logger().info(str(action))
        if action == 1:
            vel_cmd.angular.z = 1.0
        elif action == 0:
            vel_cmd.angular.z = -1.0
        else:
            vel_cmd.angular.z = 0.0

        self.pub.publish(vel_cmd)
    def is_episode_finished(self, observation):
        return abs(observation["pitch"]) > math.radians(20)
    def reset_sim(self):
        self.req = SetEntityState.Request()
        self.req.state.name = "cam_bot"
        self.req.state.pose.position = Point(x=0.0, y=0.0, z=0.0)
        self.req.state.pose.orientation = Quaternion(w=1.0,x=0.0,y=0.0,z=0.0)
        self.req.state.reference_frame = "world"
        self.make_action(2)
        self.get_logger().info(f"Kp: {self.kp}, steps: {self.steps}")
        self.steps = 0
        _ = self.client.call(self.req)
        time.sleep(0.2)
    def correction(self):
        dt = 1/self.callback_hz
        observation = self.get_observation()
        #self.get_logger().info(str(observation["pitch"] > math.radians(60)))
        if self.is_episode_finished(observation):
            self.reset_sim()
            #Pause HERE
            #_ = self.pause.call(Empty.Request())
            
            #time.sleep(0.3)
            #_ = self.unpause.call(Empty.Request())

            #UNPAUSE HERE
            #time.sleep(0.1)
            #self.kp += 0.1
            
            
        error = self.setpoint - observation["pitch"]
        error_derivative = (error - self.prev_error) / dt
        self.error_integral += error * dt

        correction = self.kp * error + self.ki * self.error_integral + self.kd * error_derivative

        self.prev_error = error

        action = self.sigmoid(correction)
        action = np.round(action).astype(np.int32)
        self.make_action(action)
        
        self.steps += 1
    
    
        


def main():
    rclpy.init()
    
    node = PID_control()#https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
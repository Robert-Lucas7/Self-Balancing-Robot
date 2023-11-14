#This is the main control cycle, the control code is run at 20HZ.

import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, Point
from std_srvs.srv import Empty
from std_msgs.msg import Int32 #sensor_msgs/msg/Imu
from sensor_msgs.msg import Imu
import math 
import numpy as np
from gazebo_msgs.srv import DeleteEntity, SpawnEntity, SetEntityState
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy



import xacro
import os
from ament_index_python.packages import get_package_share_directory
import time

class PausePhysics(Node):
    def __init__(self): #Check if entity exists first
        super().__init__("minimum_pause")

        self.client = self.create_client(Empty, "/pause_physics")
        while not self.client.wait_for_service():
            self.get_logger().info("Service not available, waiting again...")

        self.req = Empty.Request()
        
    def send_request(self):
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
class UnpausePhysics(Node):
    def __init__(self): #Check if entity exists first
        super().__init__("minimum_unpause")

        self.client = self.create_client(Empty, "/unpause_physics")
        while not self.client.wait_for_service():
            self.get_logger().info("Service not available, waiting again...")

        self.req = Empty.Request()
        
    def send_request(self):
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
class ResetSim(Node):
    def __init__(self): #Check if entity exists first
        super().__init__("minimum_reset_client")

        self.client = self.create_client(SetEntityState, "/set_entity_state")
        while not self.client.wait_for_service():
            self.get_logger().info("Service not available, waiting again...")

        self.req = SetEntityState.Request()
        self.req.state.name = "cam_bot"
        self.req.state.pose.position = Point(x=0.0, y=0.0, z=0.1)
        self.req.state.pose.orientation = Quaternion(w=1.0,x=0.0,y=0.0,z=0.0)
        self.req.state.reference_frame = "world"

    
    def send_request(self):
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


class ResetSim(Node):
    def __init__(self): #Check if entity exists first
        super().__init__("minimum_reset_client")

        self.client = self.create_client(SetEntityState, "/set_entity_state")
        while not self.client.wait_for_service():
            self.get_logger().info("Service not available, waiting again...")

        self.req = SetEntityState.Request()
        self.req.state.name = "cam_bot"
        self.req.state.pose.position = Point(x=0.0, y=0.0, z=0.1)
        self.req.state.pose.orientation = Quaternion(w=1.0,x=0.0,y=0.0,z=0.0)
        self.req.state.reference_frame = "world"

    
    def send_request(self):
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class IMUSubscriber(Node):
    def __init__(self):
        super().__init__("imu_sub")
        self.qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10
        )
        self.sub = self.create_subscription(Imu, 'imu/data', self.new_imu_data,self.qos_profile)
        self.latest_msg = Imu()
        self.prev_msg = Imu()
        self.latest_imu_data = {
            "roll" : 0.0,
            "pitch" : 0.0,
            "yaw" : 0.0,
            "angular_y":0.0
        }
        
    
    def is_the_same_observation(self):
        return self.latest_msg == self.prev_msg
    
    def new_imu_data(self, msg):
        self.prev_msg = self.latest_msg
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

    def get_latest_observation(self): 
        self.temp = self.latest_msg
        roll, pitch, yaw = self.euler_from_quaternion(self.temp.orientation.x,self.temp.orientation.y,self.temp.orientation.z,self.temp.orientation.w )
        self.latest_imu_data["roll"] = roll
        self.latest_imu_data["pitch"] = pitch
        self.latest_imu_data["yaw"] = yaw
        self.latest_imu_data["angular_y"] = self.latest_msg.angular_velocity.y
        #self.get_logger().info(str(self.latest_imu_data["angular_y"]))
        return self.latest_imu_data
    
    
class CMD_Vel_Pub(Node):
    def __init__(self):
        super().__init__("cmd_vel_pub")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

    def publish(self, action):
        msg = Twist()
        if action == 0:
            msg.angular.z = 1.5
        else:
            msg.angular.z = -1.5
        self.pub.publish(msg)

class SBR(Node):
    #============================= CURRENT PROBLEM =========================================
    # THERE IS NO IMU CALLBACK BETWEEN ENDING AN EPISODE AND STARTING THE NEXT ONE (without the while prev == observation loop (THE BIG BODGE))
    
    def __init__(self):
        super().__init__("main_control")
        self.resetSim = ResetSim()
        self.imu_sub = IMUSubscriber()
        self.cmd_vel_pub = CMD_Vel_Pub()
        self.pause = PausePhysics()
        self.unpause = UnpausePhysics()
        # self.NUM_ANGLE_BINS = 9
        # self.NUM_VELOCITY_BINS = 5
        self.NUM_BINS = 15

        self.NUM_EPISODES = 2000
        self.ANGLE_LIMIT = math.radians(12)
        self.VELOCITY_LIMIT = 2
        self.DISCOUNT_FACTOR = 0.95
        self.LEARNING_RATE = 0.1

        # self.angle_states = np.linspace(-self.ANGLE_LIMIT, self.ANGLE_LIMIT, self.NUM_ANGLE_BINS)
        # self.velocity_states = np.linspace(-self.VELOCITY_LIMIT, self.VELOCITY_LIMIT, self.NUM_VELOCITY_BINS-1)
        self.bins = [
            np.linspace(-self.ANGLE_LIMIT, self.ANGLE_LIMIT, self.NUM_BINS),
            np.linspace(-self.VELOCITY_LIMIT, self.VELOCITY_LIMIT, self.NUM_BINS)
        ]
        self.epsilon = 0.99
        self.EPSILON_DECAY = 0.993#0.996 #This gives epsilon e < 0.1 when episodes > 571
        self.Q_table = np.random.uniform(low=0, high=1, size=([self.NUM_BINS] * 2 +[2]))  #np.zeros((self.NUM_ANGLE_BINS, self.NUM_VELOCITY_BINS, 2)) #2 actions
        self.all_rewards = []
        self.observation = {}
        self.load_q_table()
        print(self.Q_table)
        self.prev_obs = {}
        self.observation = {}
        

    def go(self, exploit=False):
        
        
        time.sleep(5)
        
        try:
            for episode in range(1,2000):
                
                self.resetSim.send_request()
                
                time.sleep(1)
                self.pause.send_request()
                #sometimes this is not spinning ???
                #BELOW IS A BODGE OF CODE to fix above problem
                while self.prev_obs == self.observation:
                    rclpy.spin_once(self.imu_sub)
                    self.observation = self.imu_sub.get_latest_observation()
                ep_rewards = 0
                timesteps = 0
                self.unpause.send_request()
                current_state = self.discretise_observation()
                while not self.is_episode_finished():
                    # current_state = self.discretise_observation()
                    self.unpause.send_request()
                    action =np.argmax(self.Q_table[current_state])
                    if np.random.random() < self.epsilon and not exploit:
                        action = random.randint(0,1)
                    self.cmd_vel_pub.publish(action)
                    rclpy.spin_once(self.imu_sub)
                    timesteps += 1
                    self.observation = self.imu_sub.get_latest_observation()
                    self.pause.send_request()
                    new_state = self.discretise_observation()
                    reward = self.get_reward(current_state, new_state)#self.get_reward(new_state[0])
                    ep_rewards += reward
                    # lr = self.get_learning_rate(episode+1)
                    self.Q_table[current_state + (action, )] = (1 - self.LEARNING_RATE) * self.Q_table[current_state + (action, )] + self.LEARNING_RATE * (reward + self.DISCOUNT_FACTOR * np.max(self.Q_table[new_state]))
                    current_state = new_state
                self.get_logger().info(f"{episode}| epsilon: {self.epsilon}| Steps: {timesteps}")
                self.epsilon = max(0.01, self.epsilon * self.EPSILON_DECAY)
                self.prev_obs = self.observation.copy()
                self.all_rewards.append(ep_rewards)
                if episode % 25 == 0 and not exploit:
                    self.get_logger().info(f'LAST 100 AVG: {str(sum(self.all_rewards[-100:])/100)}')
                    self.get_logger().info(f'Saving the Q-Table to a file')
                    self.save_q_table()
                
        finally:
            pass


    def load_q_table(self):
        table = []
        q_pairs = []
        if os.path.isfile('/home/rob/sbr_data/q_table/q_table.txt'):
            with open('/home/rob/sbr_data/q_table/q_table.txt', 'r') as f:
                count = 1
                q_pairs = []
                for line in f:
                    
                    split = line.split(',')
                    q_pairs.append([float(split[0]), float(split[1].strip('\n'))])
                    if count % self.NUM_BINS == 0:
                        table.append(q_pairs)
                        q_pairs = []
                    count += 1
                    
            #self.Q_table = np.array(q_pairs)
            self.Q_table = np.array(table)           
    def save_q_table(self):
        with open('/home/rob/sbr_data/q_table/q_table.txt', 'w') as f:
            for i in self.Q_table:
                
                for j in i:
                    f.write(f'{j[0]},{j[1]}\n')
                

    def discretise_observation(self):#atm only using pitch (will need to change as the robot may have unintended rotation due to the surface it is on)
        return tuple([np.digitize(self.observation["pitch"], self.bins[0] ) - 1, np.digitize(self.observation["angular_y"], self.bins[1]) - 1])#(np.digitize(self.observation["pitch"], self.angle_states), np.digitize(self.observation["angular_y"], self.velocity_states))
    
    def choose_action(self, current_state):
        if np.random.random() < self.epsilon:
            return random.randint(0,1)
        else:
            return np.argmax(self.Q_table[current_state])
        
    # def get_learning_rate(self, episode):
    #     return max(0.01, min(1.0, 1.0 - math.log10((episode) / 25)))

    def is_episode_finished(self):
        return abs(self.observation["pitch"]) > self.ANGLE_LIMIT or abs(self.observation["angular_y"]) > self.VELOCITY_LIMIT

    def get_reward(self, previous_angle_state, new_angle_state):
        midpoint = (self.NUM_BINS + 1)/2
        if abs(midpoint - new_angle_state[0]) < abs(midpoint - previous_angle_state[0]):
            return 1
        else:
            return -1
            

def main(args = None):
    rclpy.init(args=args)
    control_node = SBR()
    control_node.go(exploit=False)
    rclpy.shutdown()



if __name__ == "__main__":
    main()
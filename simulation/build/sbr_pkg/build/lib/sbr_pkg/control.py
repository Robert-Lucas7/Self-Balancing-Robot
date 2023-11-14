#This is the main control cycle, the control code is run at 20HZ.

import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from std_msgs.msg import Int32 #sensor_msgs/msg/Imu
from sensor_msgs.msg import Imu
import math 
import numpy as np
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
import xacro
import os
from ament_index_python.packages import get_package_share_directory
import time

class ResetSim(Node):
    def __init__(self):
        super().__init__("minimum_reset_client")
        self.client = self.create_client(Empty, "/reset_world")
        while not self.client.wait_for_service():
            self.get_logger().info("Service not available, waiting again...")
        self.req = Empty.Request()
    
    def send_request(self):
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
class SpawnModel(Node):
    def __init__(self):
        super().__init__("min_spawn")
        self.client = self.create_client(SpawnEntity, "/spawn_entity")
        while not self.client.wait_for_service():
            self.get_logger().info("Service not available, waiting again...")
        self.req = SpawnEntity.Request()
    
    def send_request(self):
        urdf = os.path.join(get_package_share_directory('sbr_pkg'),'urdf', 'two_wheeled_copied.xacro')
        self.get_logger().info(urdf)
        doc = xacro.process_file(urdf)
        robot_desc = doc.toprettyxml(indent='   ')

        self.req.name = "my_bot"
        self.req.xml = robot_desc
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("SPAWNED")
        return self.future.result()
    
class DeleteModel(Node):
    def __init__(self):
        super().__init__("min_delete")
        self.client = self.create_client(Empty, "/delete_entity")
        while not self.client.wait_for_service():
            self.get_logger().info("Service not available, waiting again...")
        self.req = DeleteEntity.Request()
    
    def send_request(self):
        self.req.name = "my_bot"
        self.future = self.client.call_async(self.req)
        self.get_logger().info("deelting...")
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()
    
class PausePhysics(Node):
    def __init__(self):
        super().__init__("minimum_pause_client")
        self.client = self.create_client(Empty, "/pause_physics")
        while not self.client.wait_for_service():
            self.get_logger().info("Service not available, waiting again...")
        self.req = Empty.Request()
    
    def send_request(self):
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("Reset World!")
        return self.future.result()
    
class UnpausePhysics(Node):
    def __init__(self):
        super().__init__("min_pause")
        self.client = self.create_client(Empty, "/unpause_physics")
        while not self.client.wait_for_service():
            self.get_logger().info("Service not available, waiting again...")
        self.req = Empty.Request()
    
    def send_request(self):
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("Reset World!")
        return self.future.result()
'''
class SBRNode(Node):
    def __init__(self):
        super().__init__("Q_Learning_Control")

        #No cart position limit yet (estimate distance from origin with linear velocity/acceleration)
        self.NUM_ANGLE_BINS = 10
        self.NUM_VELOCITY_BINS = 5
        self.NUM_EPISODES = 500
        self.ANGLE_LIMIT = math.radians(20)
        self.VELOCITY_LIMIT = 2
        self.DISCOUNT_FACTOR = 0.9

        self.angle_states = np.linspace(-self.ANGLE_LIMIT, self.ANGLE_LIMIT, self.NUM_ANGLE_BINS-1)
        self.velocity_states = np.linspace(-self.VELOCITY_LIMIT, self.VELOCITY_LIMIT, self.NUM_VELOCITY_BINS-1)
        
        self.latest_imu_reading = Imu()
        self.epsilon = 0.99
        self.epsilon_decay = 0.99
        self.Q_table = np.zeros((self.NUM_ANGLE_BINS, self.NUM_VELOCITY_BINS, 2)) #2 actions
        self.all_rewards = []

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.new_imu_data,1)
        self.reset_client = self.create_client(Empty, "/reset_world")

        
    def reset_model(self):

        self.get_logger().info("AHH")
        while not self.reset_client.wait_for_service(timeout_sec = None):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info("WOO")
        self.request = Empty.Request()
        self.get_logger().info("NOOOOO")
        self.future = self.reset_client.call_async(self.request)
        self.get_logger().info("AFTER CALL")
        #rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("SPIINNNING")
        return self.future.result()

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

    def discretise_observation(self):#atm only using pitch (will need to change as the robot may have unintended rotation due to the surface it is on)
        _, pitch, __ = self.euler_from_quaternion(self.latest_imu_reading.orientation.x,self.latest_imu_reading.orientation.y, self.latest_imu_reading.orientation.z, self.latest_imu_reading.orientation.w)
        self.get_logger().info(f"Pitch is: {pitch}")
        return (np.digitize(math.radians(pitch), self.angle_states), np.digitize(self.latest_imu_reading.angular_velocity.y, self.velocity_states))
    
    def choose_action(self, current_state):
        if np.random.random() < self.epsilon:
            return random.randint(0,1)
        else:
            return np.argmax(self.Q_table[current_state])
        
    def get_learning_rate(self, episode):
        return max(0.01, min(1.0, 1.0 - math.log10((episode + 1) / 25)))

    def is_episode_finished(self):
        _, pitch, __ = self.euler_from_quaternion(self.latest_imu_reading.orientation.x,self.latest_imu_reading.orientation.y, self.latest_imu_reading.orientation.z, self.latest_imu_reading.orientation.w)
        return abs(math.radians(pitch)) > self.ANGLE_LIMIT or abs(self.latest_imu_reading.angular_velocity.y) > self.VELOCITY_LIMIT

    def get_reward(self, new_angle_state):
        if new_angle_state == 0 or new_angle_state == self.NUM_ANGLE_BINS - 1:
            return -1
        else:
            return 1

    def move_robot(self, action):
        msg = Twist()
        if action == 0:
            msg.angular.z = 0.7
        else:
            msg.angular.z = -0.7
        self.cmd_vel_pub.publish(msg)
        

    def learn(self):
        time.sleep(4)
        for episode in range(500):
            self.get_logger().info("New loop")
            #Reset robot model
            self.reset_model()
            episode_rewards = 0
            while(not self.is_episode_finished()):
                current_state = self.discretise_observation()
                action = self.choose_action(current_state)
                self.get_logger().info(f"Current state: {current_state}")
                #Perform action (publish twist message to /cmd_vel)
                self.move_robot(action)

                new_state = self.discretise_observation()
                reward = self.get_reward(new_state[0])
                episode_rewards += reward
                lr = self.get_learning_rate(episode+1)
                self.Q_table[current_state][action] = self.Q_table[current_state][action] + lr * (reward + self.DISCOUNT_FACTOR * np.max(self.Q_table[new_state]) - self.Q_table[current_state][action])
            
            self.epsilon = max(0.01, self.epsilon * self.epsilon_decay)
            self.all_rewards.append(episode_rewards)
            self.get_logger().info(f"{episode}| Episode rewards: {episode_rewards}")

    def control_cycle(self):
        #Here the observation will be made (get the latest message posted to /imu/data topic and an action will be chosen based on it)
        #self.q_learning.choose_action(self.latest_imu_reading)
        #self.get_logger().info("LOGGER MESSAGE")
        x = self.latest_imu_reading.orientation.x
        y = self.latest_imu_reading.orientation.y
        z = self.latest_imu_reading.orientation.z
        w = self.latest_imu_reading.orientation.w
        roll,pitch,yaw = self.euler_from_quaternion(x,y,z,w)
        self.get_logger().info(f'Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}')
        
        self.reset_model()

    def new_imu_data(self, msg): #only need orientation (in RPY) and angular velocity atm.
        self.latest_imu_reading = msg
    '''
class IMUSubscriber(Node):
    def __init__(self):
        super().__init__("imu_sub")
        self.sub = self.create_subscription(Imu, 'imu/data', self.new_imu_data,1)
        self.latest_msg = Imu()
        self.latest_imu_data = {
            "roll" : 0.0,
            "pitch" : 0.0,
            "yaw" : 0.0,
            "angular_y":0.0
        }

    
    def new_imu_data(self, msg):
        self.latest_msg = msg
        self.get_logger().info(f'x: {self.latest_msg.orientation.x}, y: {self.latest_msg.orientation.y}, z: {self.latest_msg.orientation.z}')

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
        roll, pitch, yaw = self.euler_from_quaternion(self.latest_msg.orientation.x,self.latest_msg.orientation.y,self.latest_msg.orientation.z,self.latest_msg.orientation.w )
        #self.get_logger().info(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}, Angular: {self.latest_msg.angular_velocity.y}")
        self.latest_imu_data["roll"] = roll
        self.latest_imu_data["pitch"] = pitch
        self.latest_imu_data["yaw"] = yaw
        self.latest_imu_data["angular_y"] = self.latest_msg.angular_velocity.y
        return self.latest_imu_data
    
class CMD_Vel_Pub(Node):
    def __init__(self):
        super().__init__("cmd_vel_pub")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

    def publish(self, action):
        msg = Twist()
        if action == 0:
            msg.angular.z = 0.7
        else:
            msg.angular.z = -0.7
        self.pub.publish(msg)

class SBR(Node):
    def __init__(self):
        super().__init__("main_control")
        self.resetSim = ResetSim()
        self.imu_sub = IMUSubscriber()
        self.cmd_vel_pub = CMD_Vel_Pub()
        self.pause = PausePhysics()
        self.unpause = UnpausePhysics()


        self.NUM_ANGLE_BINS = 10
        self.NUM_VELOCITY_BINS = 5
        self.NUM_EPISODES = 500
        self.ANGLE_LIMIT = math.radians(20)
        self.VELOCITY_LIMIT = 2
        self.DISCOUNT_FACTOR = 0.9

        self.angle_states = np.linspace(-self.ANGLE_LIMIT, self.ANGLE_LIMIT, self.NUM_ANGLE_BINS-1)
        self.velocity_states = np.linspace(-self.VELOCITY_LIMIT, self.VELOCITY_LIMIT, self.NUM_VELOCITY_BINS-1)
        
        self.epsilon = 0.99
        self.epsilon_decay = 0.99
        self.Q_table = np.zeros((self.NUM_ANGLE_BINS, self.NUM_VELOCITY_BINS, 2)) #2 actions
        self.all_rewards = []
        self.observation = {}

        #self.spawn = SpawnModel()
        #self.delete = DeleteModel()
        

    def learn(self):
       
        
        actual_ep = 1
        for episode in range(5000):
            #Reset robot model
            #self.pause.send_request()
            
            #self.unpause.send_request()
            time.sleep(0.05)
            episode_rewards = 0
            rclpy.spin_once(self.imu_sub)
            self.observation = self.imu_sub.get_latest_observation()
            while(not self.is_episode_finished()):
                current_state = self.discretise_observation()
                action = self.choose_action(current_state)
                
                #Perform action (publish twist message to /cmd_vel)
                self.cmd_vel_pub.publish(action) #move_robot()
                rclpy.spin_once(self.imu_sub)
                self.observation = self.imu_sub.get_latest_observation()

                new_state = self.discretise_observation()
                reward = self.get_reward(new_state[0])
                episode_rewards += reward
                lr = self.get_learning_rate(episode+1)
                self.Q_table[current_state][action] = self.Q_table[current_state][action] + lr * (reward + self.DISCOUNT_FACTOR * np.max(self.Q_table[new_state]) - self.Q_table[current_state][action])
            
            
            #if(episode_rewards !=0):
            self.get_logger().info(f"{episode}| Episode rewards: {episode_rewards}")
            self.epsilon = max(0.01, self.epsilon * self.epsilon_decay)
            self.all_rewards.append(episode_rewards)
            actual_ep += 1
            self.resetSim.send_request()
            

    def discretise_observation(self):#atm only using pitch (will need to change as the robot may have unintended rotation due to the surface it is on)
        return (np.digitize(self.observation["pitch"], self.angle_states), np.digitize(self.observation["angular_y"], self.velocity_states))
    
    def choose_action(self, current_state):
        if np.random.random() < self.epsilon:
            return random.randint(0,1)
        else:
            return np.argmax(self.Q_table[current_state])
        
    def get_learning_rate(self, episode):
        return max(0.01, min(1.0, 1.0 - math.log10((episode + 1) / 25)))

    def is_episode_finished(self):
        return abs(self.observation["pitch"]) > self.ANGLE_LIMIT or abs(self.observation["angular_y"]) > self.VELOCITY_LIMIT

    def get_reward(self, new_angle_state):
        if new_angle_state == 0 or new_angle_state == self.NUM_ANGLE_BINS - 1:
            return -1
        else:
            return 1

#Need to spin subscriber
    

def main(args = None):
    rclpy.init(args=args)
    control_node = SBR()
    control_node.learn()
    #rclpy.spin(control_node)
    #control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
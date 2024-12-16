from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32MultiArray, String # Bryan: For subscribing to the reddest object position
import numpy as np
import time
import json

IMAGE_WIDTH = 1400

# TODO: Add your new constants here
#ALL CONSTANTS WITH TODO NEXT TO THEM ARE NOT SET YET 1 IS A RANDOM CONSTANT
TIMEOUT = 1 #TODO threshold in timer_callback
SEARCH_YAW_VEL = .5 #TODO searching constant
TRACK_FORWARD_VEL = .6 #TODO tracking constant
KP = 1 #TODO proportional gain for tracking

LAST_X = -1.0

class State(Enum):
    SEARCH = 0
    TRACK = 1
    WAIT = 2

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.command_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.whispr_subscription = self.create_subscription(
            String,
            'user_query_topic',  # Replace with your topic name for queries
            self.whispr_callback,
            10
        )


        self.gpt_subscription = self.create_subscription(
            String,
            'gpt4_response_topic',
            self.gpt_callback,
            10
        )

        self.dino_subscription = self.create_subscription(
            String,
            'grounding_output',  # Topic name
            self.dino_callback,  # Callback function
            10  # QoS profile
        )
        self.json_file = ""

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = State.TRACK

        # TODO: Add your new member variables here
        self.kp = 1 # TODO
        self.best_x = 0
        self.time = 0
        self.walk = False
        self.find = False   


    def dino_callback(self, msg):
        try:
            # Parse the JSON message
            data = json.loads(msg.data)
            
            # Extract values from the JSON object
            self.walk = data.get("walk", False)
            self.find = data.get("find", False)
            self.best_x = (data.get("best_x", 0.0) - 0.5) * 2

            self.get_logger().info(
                f"Received JSON: walk={self.walk}, find={self.find}, best_x={self.best_x}"
            )
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding JSON: {e}")
                  

    def whispr_callback(self, msg):
        print(msg)
    
    def gpt_callback(self, msg):
        print(msg)

    def timer_callback(self):
        """
        Implement a timer callback that sets the moves through the state machine based on if the time since the last detection is above a threshold TIMEOUT
        """
        if self.walk:
            self.state = State.TRACK
        elif self.find:
            self.state = State.SEARCH
        else:
            self.state = State.WAIT

        yaw_command = 0.0
        forward_vel_command = 0.0

        if self.state == State.WAIT:
            yaw_command = 0.0
            forward_vel_command = 0.0

        if self.state == State.SEARCH: #part 3.1
            #yaw_command = SEARCH_YAW_VEL #* -np.sign(LAST_X)
            forward_vel_command = 0.0


        elif self.state == State.TRACK:
            print(self.best_x)
            K_p = 1.5
            yaw_command = -K_p*self.best_x
            forward_vel_command = TRACK_FORWARD_VEL
            LAST_X = self.best_x
            

        cmd = Twist()
        cmd.angular.z = yaw_command
        cmd.linear.x = forward_vel_command
        print(self.state)
        print(cmd)
        self.command_publisher.publish(cmd)

def main():
    rclpy.init()
    state_machine_node = StateMachineNode()

    try:
        rclpy.spin(state_machine_node)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        zero_cmd = Twist()
        state_machine_node.command_publisher.publish(zero_cmd)

        state_machine_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Shared message store (can be imported or accessed globally)
shared_message_store = {"message": ""}

class MemoryPublisher(Node):
    def __init__(self):
        super().__init__('memory_publisher')

        # Logger to notify that the Memory Publisher Node has started
        self.get_logger().info('Memory Publisher Node has started.')

    def publish_message(self, message):
        # Update the shared message store
        shared_message_store["message"] = message

        # DEBUG LOGGER: Uncomment to print the updated shared message
        self.get_logger().info(f"Updated shared message store: {shared_message_store['message']}")

def main(args=None):
    rclpy.init(args=args)

    # Create the memory publisher node
    memory_publisher = MemoryPublisher()

    # Keep taking user input and updating the shared message store
    try:
        while rclpy.ok():
            # Get input from the user
            user_input = input("Enter a command for GPT-4: ")

            # Exit condition
            if user_input.lower() == 'exit':
                print("Exiting the publisher.")
                break

            # Publish the input to the shared store
            memory_publisher.publish_message(user_input)

            # Allow ROS2 to process (even though no message is being published to a topic here)
            rclpy.spin_once(memory_publisher, timeout_sec=0.1)

    except KeyboardInterrupt:
        print("Interrupted by user. Exiting...")

    # Clean up and shutdown
    memory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

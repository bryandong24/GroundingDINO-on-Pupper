import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI

client = OpenAI(api_key='ENTER-YOUR-API-KEY')

class GPT4ConversationNode(Node):
    def __init__(self):
        super().__init__('gpt4_conversation_node')

        # Create a subscriber to listen to user queries
        self.subscription = self.create_subscription(
            String,
            'user_query_topic',  # Replace with your topic name for queries
            self.query_callback,
            10
        )

        # Create a publisher to send back responses
        self.publisher_ = self.create_publisher(
            String,
            'gpt4_response_topic',  # Replace with your topic name for responses
            10
        )

        self.get_logger().info('GPT-4 conversation node started and waiting for queries...')


    
    # TODO: Implement the query_callback method
    # msg is a String message object that contains the user query. You can extract the query using msg.data
    def query_callback(self, msg):
        # Extract the user query from the message using the data attribute of message
        user_query = msg.data
        self.get_logger().info(f"Received user query: {user_query}") 
        
        # Call GPT-4 API to get the response. Use the get_gpt4_response method and pass in the query
        response = self.get_gpt4_response(user_query)

        # Publish the response (as the data to a String message) using self.publisher_ and its publish method,

        response1 = String()
        response1.data = response
        self.publisher_.publish(response1)
        

        # Publish the response to the ROS2 topic 
        # (skip)
        
        # DEBUG LOGGERS: Uncomment the following line to print the query and response (you may have to change the variable names)
        
        self.get_logger().info(f"Published GPT-4 response: {response}")
        self.get_logger().info(f"ROS2 Class string response: {response1}")


    def get_gpt4_response(self, query):
        try:
            # Making the API call to GPT-4o using OpenAI's Python client
            #  give the steps you would use to spell \" HOT TO GO\" with your movement.[INFO] [1731027656.589646841] [gpt4_conversation_node]: Published GPT-4 response: bark

            # prompt = "you are a english to ros2 translator. Take the following command and use"
            prompt = "You are a english to object translator. The prompt will identify an object that it wants you to move towards. output the cleaned version of the object. for example, if the prompt is 'Move uh towards uh the redish car thank you' you need to output 'red car' but also output 'car' as well but start with 'red car'. The output should not have quotation marks and should be seperated by commas if it is more than 1. everything lower case too. If the query doesn't ask you to go to a certain place, your response should be 'no destination'"
            response = client.chat.completions.create(model="gpt-4",  # Model identifier, assuming GPT-4 is used
            messages=[
                {"role": "system", "content": prompt},
                {"role": "user", "content": query}
            ],
            max_tokens=150)  # Adjust token limit based on your requirement)

            # Extract the assistant's reply from the response
            gpt4_response = response.choices[0].message.content
            return gpt4_response

        except Exception as e:
            self.get_logger().error(f"Error calling GPT-4 API: {str(e)}")
            return "Sorry, I couldn't process your request due to an error."

def main(args=None):
    rclpy.init(args=args)

    # Create the node and spin it
    gpt4_conversation_node = GPT4ConversationNode()
    rclpy.spin(gpt4_conversation_node)

    # Clean up and shutdown
    gpt4_conversation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

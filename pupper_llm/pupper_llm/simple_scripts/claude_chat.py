import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from anthropic import Anthropic

client = Anthropic(api_key='ENTER-YOUR-API-KEY')

class GPT4ConversationNode(Node):
    def __init__(self):
        super().__init__('gpt4_conversation_node')

        # Create a subscriber to listen to user queries
        self.subscription = self.create_subscription(
            String,
            'user_query_topic',
            self.query_callback,
            10
        )

        # Create a publisher to send back responses
        self.publisher_ = self.create_publisher(
            String,
            'gpt4_response_topic',
            10
        )

        self.get_logger().info('Claude conversation node started and waiting for queries...')

    def query_callback(self, msg):
        user_query = msg.data
        self.get_logger().info(f"Received user query: {user_query}") 
        
        # Call GPT-4 API to get the response. Use the get_gpt4_response method and pass in the query
        response = self.get_gpt4_response(user_query)

        response_msg = String()
        response_msg.data = response

        self.publisher_.publish(response_msg)
        
        self.get_logger().info(f"Published Claude response: {response}")



    def get_gpt4_response(self, query):
        try:
            # Making the API call to GPT-4o using OpenAI's Python client
            #  give the steps you would use to spell \" HOT TO GO\" with your movement.[INFO] [1731027656.589646841] [gpt4_conversation_node]: Published GPT-4 response: bark

            # prompt = "you are a english to ros2 translator. Take the following command and use"
            prompt = "You are a english to object translator. The prompt will identify an object that it wants you to move towards. output the cleaned version of the object. for example, if the prompt is 'Move uh towards uh the redish car thank you' you need to output 'red car' but also output 'car' as well but start with 'red car'. The output should not have quotation marks and should be seperated by commas if it is more than 1. everything lower case too. If the query doesn't ask you to go to a certain place, your response should be 'no destination'"
            
            message = client.messages.create(
                model="claude-3-sonnet-20240229",
                max_tokens=2048,
                system=prompt,
                messages=[
                    {
                        "role": "user",
                        "content": query
                    }
                ]
            )
            
            return message.content[0].text

        except Exception as e:
            self.get_logger().error(f"Error calling Claude API: {str(e)}")
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

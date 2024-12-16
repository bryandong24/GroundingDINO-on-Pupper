import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import json
import time
import argparse
import torch
from PIL import Image as PILImage
from io import BytesIO

# Import functions from inference_on_a_image.py
from inference_on_a_image import load_model, load_image, get_grounding_output

class GroundingNode(Node):
    def __init__(self):
        super().__init__('grounding_node')
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.text_sub = self.create_subscription(
            String,
            'gpt4_response_topic',
            self.text_callback,
            10
        )
        self.publisher = self.create_publisher(String, 'grounding_output', 10)
        self.last_text_time = None
        self.last_text = None
        self.image = None
        self.timer = self.create_timer(1, self.process_and_publish) # Runs every second

    # modify these values to change how the model runs
    parser = argparse.ArgumentParser("Grounding DINO example", add_help=True)
    parser.add_argument("--config_file", "-c", type=str, required=True, help="path to config file")
    parser.add_argument(
        "--checkpoint_path", "-p", type=str, required=True, help="path to checkpoint file"
    )
    parser.add_argument("--image_path", "-i", type=str, required=True, help="path to image file")
    parser.add_argument("--text_prompt", "-t", type=str, required=True, help="text prompt")
    parser.add_argument(
        "--output_dir", "-o", type=str, default="outputs", required=True, help="output directory"
    )

    parser.add_argument("--box_threshold", type=float, default=0.3, help="box threshold")
    parser.add_argument("--text_threshold", type=float, default=0.25, help="text threshold")
    parser.add_argument("--token_spans", type=str, default=None, help=
                        "The positions of start and end positions of phrases of interest. \
                        For example, a caption is 'a cat and a dog', \
                        if you would like to detect 'cat', the token_spans should be '[[[2, 5]], ]', since 'a cat and a dog'[2:5] is 'cat'. \
                        if you would like to detect 'a cat', the token_spans should be '[[[0, 1], [2, 5]], ]', since 'a cat and a dog'[0:1] is 'a', and 'a cat and a dog'[2:5] is 'cat'. \
                        ")

    parser.add_argument("--cpu-only", action="store_true", help="running on cpu only!, default=False")
    args = parser.parse_args()

    # cfg
    config_file = args.config_file  # change the path of the model config file
    checkpoint_path = args.checkpoint_path  # change the path of the model
    image_path = args.image_path
    text_prompt = args.text_prompt
    output_dir = args.output_dir
    box_threshold = args.box_threshold
    text_threshold = args.text_threshold
    token_spans = args.token_spans


    def image_callback(self, msg):
        self.image = PILImage.open(BytesIO(msg.data)).convert("RGB")

    def text_callback(self, msg):
        self.last_text = msg.data
        self.last_text_time = time.time()

    def process_and_publish(self):
        if self.image is None or self.last_text is None:
            return

        # Load model and process image
        model = load_model(config_file, checkpoint_path, cpu_only=True)
        image_pil, image = load_image(self.image)
        boxes_filt, pred_phrases = get_grounding_output(
            model, image, self.last_text, box_threshold, text_threshold, cpu_only=True
        )

        # Determine walk and find
        walk = False
        find = False
        if boxes_filt.size(0) > 0:
            max_prob = max([float(phrase.split('(')[-1][:-1]) for phrase in pred_phrases])
            walk = max_prob >= 0.6
            if max_prob <= 0.6 and (time.time() - self.last_text_time) > 30:
                find = True
        # if the text has not been updated in 30 seconds, reset the walk and find flags
        if time.time() - self.last_text_time > 30:
            walk = False
            find = False
        # Publish result
        result = {
            "walk": walk,
            "find": find,
            "best_x": 0.8  # Placeholder for actual calculation
        }
        self.publisher.publish(String(data=json.dumps(result)))

def main(args=None):
    rclpy.init(args=args)
    node = GroundingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
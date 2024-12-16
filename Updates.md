- Integrated the pupper_llm project with lab_7.
	- Added the simple_chat_gopt
- added two subscribers (subscribes to the ) to lab_7.py

### How to run
1. `bash run.sh`
2. `python3 lab_7.py`
3. cd to simple scripts
4. `python3 simple_gpt_chat.py`
5. `python3 whisper_ping.py`
6. Say `exit` when you're done?

### Future tasks
- Set up grounding dino server
- Set up Zenoh bridge
- Set up a separate state where it's not moving when it doesn't have a command
- Use hailo_detection.py to move. Find which bounding box on hailo_detection 
---

### Node in James' server
Create a new node.
Take the GroundingDINO output. 
GPT response topic name: `gpt4_response_topic`
Take GPT response, parse, then output the x direction to another topic.


Publishes to topic: `DINO_x_direction_topic.  

Published information in the `.JSON` format.
```json
{
	"walk" : true, // Move if true, don't move if false. 
	"find" : true, // If true, call search state
	"best_x" : 0.8, // must be normalized to be between [-1, 1]
}
```



In the lab_7.py file have the `StateMachineNode(Node)` subscribe to the x directions and move in that direction using current setup.





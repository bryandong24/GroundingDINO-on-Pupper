The goal of our project was to support zero-shot vision models on the pupper. We ran Grounding DINO which allowed us to process the images from the camera, and return a target bounding box to lock onto. Since Grounding DINO requires quite a bit of compute, we used a Zenoh bridge to connect ROS2 nodes between our robot and the server.

  <iframe style="border: 1px solid rgba(0, 0, 0, 0.1);" width="800" height="450" src="https://embed.figma.com/board/rKJtkOiUa4A0HkFyTeKj5N/Untitled?node-id=0-1&embed-host=share" allowfullscreen></iframe>

[Link to slides](https://docs.google.com/presentation/d/1DscjwsaXE5AMdKXYatM0UojeO6OOYxdLWSe9gKqW-aA/edit?usp=sharing) (Videos and pictures in slides)

> Note, currently the project is set up to only run from command line inputs, *not* voice inputs because OpenAI was down on demo day. 
  
### Dependencies / Requirements
- A server with a GPU for the inference model. Ideally H100 running on x86 server. (If not an x86 server, reinstall the correct zenoh plugin [here](https://download.eclipse.org/zenoh/zenoh-plugin-ros2dds/latest/). )
- Some form of NAT traversal network like Tailscale or Zerotier if the server isn't being run on the same network.
- [The server files](https://drive.google.com/drive/folders/1WhypdyoNzZovLyZQYCk08r78Wch0_sGF?usp=sharing)

### How to run
1. On the server, pull [this repository](https://github.com/bryandong24/GroundingDINO-Server).
2. Navigate to the `GroundingDINO-Server/zenoh-bridge-standalone/` and execute the binary using `zenoh-bridge-ros2dds -e tcp/<robot-ip>:7447`. Be sure to replace `<robot-ip>` with your actual robot ip.
3. Navigate to `GroundingDino-Server/demo/` and execute the python file `dino_node_ros2_combined.py`
4. On the robot, `git pull` this repository.\
5. Navigate to `GroundingDINO-on-pupper/Pupper/pupper_llm/pupper_llm/simple_scripts/claude_chat.py`
6.  On line 6 change `client = Anthropic(api_key='ENTER-YOUR-API-KEY')` to contain a functional Anthropic API key.
5. After you `cd` into the repository, type in `bash run.sh`
6. Type in `tmux attach -t publish` to open up the terminal where you give the robot commands.a

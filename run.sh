tmux new-session -d -s initial_commands 'bash -c "source ~/.bashrc; ros2 launch ~/CS123FinalProject/lab_7.launch.py & ros2 launch ~/CS123FinalProject/foxglove_bridge foxglove_bridge_launch.xml & python ~/CS123FinalProject/hailo_detection.py"'
echo "To attach to the 'initial_commands' session, use: tmux attach -t initial_commands"

sleep 5
# tmux new-session -d -s gpt 'python3 ~/CS123FinalProject/pupper_llm/pupper_llm/simple_scripts/simple_gpt_chat.py'
# echo "To attach to the 'gpt' session, use: tmux attach -t gpt"

tmux new-session -d -s whisper 'python3 ~/CS123FinalProject/pupper_llm/pupper_llm/simple_scripts/whisper_ping.py'
echo "To attach to the 'whisper' session, use: tmux attach -t whisper"

tmux new-session -d -s claude 'python3 ~/CS123FinalProject/pupper_llm/pupper_llm/simple_scripts/claude_chat.py'
echo "To attach to the 'claude' session, use: tmux attach -t claude"

tmux new-session -d -s zenoh '~/zenoh-bridge/zenoh-bridge-ros2dds'
echo "To attach to the 'zenoh' session, use: tmux attach -t zenoh"

tmux new-session -d -s lab7 'python3 ~/CS123FinalProject/lab_7.py'
echo "To attach to the 'lab7' session, use: tmux attach -t lab7"

# Optionally, attach to one of the sessions to view output
# ttmmux attach-session -t mysession1
tmux new-session -d -s publish 'python3 /home/pi/CS123FinalProject/pupper_llm/pupper_llm/Robot_Commands/command_line_publisher.py'
echo "To attach to the 'publish' session, use: tmux attach -t publish"
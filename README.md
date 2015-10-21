# robot voice control

Various tools to control the robot with your voice.

## message_control node

Message-based NL control: This node Listens to the specified topic for parsed
natural language commands. The control of the robot is all done through topics: if
it hears a particular command this node will output the given command token on 
the particular topic. This supports various toke types: Strings, Ints, Floats, etc..

All control definitions happens with ROS parameters, which specify the topics, inputs, and outputs.
At startup, the node loads the ROS parameter that specify which topics and messages
should be mapped to which command.  See 
[this config file](robot_voice_control/config/allegro_hand_control.yaml) 
for an example and more details on specifications of the parameter file.

Note that two commands cannot be shared between different topics (e.g,
'stop' cannot map to different commands on two topics).


## Useful package links:

  - [Pocketsphinx](https://github.com/felixduvallet/pocketsphinx)

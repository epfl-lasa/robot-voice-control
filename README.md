# robot voice control

Various tools to control the robot with your voice.

So far the contents are:

 - robot_voice_control: A ROS package, described below
 - state_machine_template: A ROS package with some SMACH (state machine) code.

### message_control node

Message-based NL control:
behaves essentially as a If-This-(Input)-Then-That-(Command),
where Input is a natural language string, and the command is an
arbitrary token recognized by the robot.

[The node] (robot_voice_control/nodes/message_control.py)
listens to the specified topic for parsed
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


### Useful package links:

  - [Pocketsphinx](https://github.com/felixduvallet/pocketsphinx)

### Creating a new language model:

See the README in [pocketsphinx](https://github.com/felixduvallet/pocketsphinx)
for instructions on creating your own language model.


[![Build Status](https://travis-ci.org/epfl-lasa/robot-voice-control.svg?branch=master)](https://travis-ci.org/epfl-lasa/robot-voice-control)

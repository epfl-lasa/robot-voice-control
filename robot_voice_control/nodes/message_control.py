#!/usr/bin/env python

# Node for message-based NL control. Listens to the specified topic for parsed
# natural language. The control of the robot is all done through messages: if
# it hears a particular command this node will output a given string on a
# particular topic.
#
# At startup, loads the ROS parameter that specify which topics and messages
# should be mapped to which command.

import rospy
from std_msgs.msg import String

class LanguageToMessageTranslator(object):

    def __init__(self):

        nl_command_topic = '/nl_command_parsed'

        # map of nl_command -> (topic, message)
        self.nl_command_map = {'home': ('/allegroHand/lib_cmd', 'home'),
                               'pinch': ('/allegroHand/lib_cmd', 'pinch_it'),
                               'gravity compensation': ('/allegroHand/lib_cmd',
                                                        'gravcomp')
                               }
        rospy.Subscriber(nl_command_topic, String, self.nl_command_callback)

        rospy.loginfo('NL control running, listening to topic: {}.'.format(nl_command_topic))

    def nl_command_callback(self, msg):
        command = msg.data

        if command in self.nl_command_map:
            rospy.loginfo('Heard command: {}'.format(command))
            (topic, message) = self.nl_command_map[command]
            rospy.loginfo('Sending command; topic[{}], message[{}]'.format(
                topic, message))

            # NOTE It may be useful to start all the publishers at startup.
            publisher = rospy.Publisher(topic, String, queue_size=1)
            publisher.publish(message)
        else:
            rospy.loginfo('Unknown NL command: {}'.format(command))


def run():
    rospy.init_node('message_control')

    translator = LanguageToMessageTranslator()

    rospy.spin()


if __name__ == '__main__':
    run()

__author__ = 'felixd'

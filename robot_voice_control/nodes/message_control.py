#!/usr/bin/env python

# Node for message-based NL control. Listens to the specified topic for parsed
# natural language. The control of the robot is all done through messages: if
# it hears a particular command this node will output a given string on a
# particular topic.
#
# At startup, loads the ROS parameter that specify which topics and messages
# should be mapped to which command.

import rospy
from collections import defaultdict

import std_msgs
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

    def load_nl_command_map(self, control_param_name):

        if not rospy.has_param(control_param_name):
            rospy.logerr('Cannot proceed without the control parameter: {}'.format(control_param_name))
            rospy.signal_shutdown('Cannot proceed without parameter.')

        param = rospy.get_param(control_param_name)
        rospy.loginfo('param: {}'.format(param))
        assert 'topics' in param

        # Convert a list of dictionaries to a list of tuples.
        topics_and_types = [x.items()[0] for x in param['topics']]

        for (param_topic_name, topic_type_str) in topics_and_types:
            if not rospy.has_param(param_topic_name):
                rospy.logerr('Expected parameter {}'.format(param_topic_name))
            rospy.loginfo('Getting all command maps that go on topic {}'.format(param_topic_name))
            command_mapping = rospy.get_param(param_topic_name)

            tmp = LanguageToMessageTranslator.parse_command_mapping(
                param_topic_name, topic_type_str, command_mapping)

            self.nl_command_map.update(tmp)


        rospy.loginfo('NL control running, listening to topic: {}.'.format(nl_command_topic))

    @classmethod
    def parse_command_mapping(cls, topic_name, topic_type_str, command_mapping):
        ret = {}

        msg_t = cls.string_to_type[topic_type_str]
        if not msg_t:
            return ret

        return ret

    # This defaultdict maps a string representation of the message type
    # (what's stored inside the parameter server) to the actual message type.
    # By default it returns None (instead of raising an exception) if the
    # message type is unknown.
    string_to_type = defaultdict(lambda: None,
                                 [('String', std_msgs.msg.String),
                                  ('Int32', std_msgs.msg.Int32),
                                  ('Float32', std_msgs.msg.Float32)])

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

    ####rospy.spin()


if __name__ == '__main__':
    run()

__author__ = 'felixd'

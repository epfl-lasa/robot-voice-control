#!/usr/bin/env python

# Node for message-based NL control. Listens to the specified topic for parsed
# natural language. The control of the robot is all done through messages: if
# it hears a particular command this node will output a given string on a
# particular topic.
#
# At startup, loads the ROS parameter that specify which topics and messages
# should be mapped to which command.
#
# Note that two commands cannot be shared between different topics (e.g,
# 'stop' cannot map to different commands on two topics).

import rospy
from collections import defaultdict

import std_msgs
from std_msgs.msg import String


class LanguageToMessageTranslator(object):

    def __init__(self):

        self.nl_command_topic = '/nl_command_parsed'

        # map of nl_command -> (topic, message)
        self._nl_command_map = {}
        rospy.Subscriber(self.nl_command_topic, String, self.nl_command_callback)

    def load_nl_command_map(self, control_param_name):

        rospy.loginfo('Looking for parameter: {}'.format(control_param_name))
        if not rospy.has_param(control_param_name):
            rospy.logerr('Cannot proceed without the control parameter: {}'.
                         format(control_param_name))
            rospy.signal_shutdown('Cannot proceed without parameter.')

        param = rospy.get_param(control_param_name)
        assert 'topics' in param

        # Convert a list of dictionaries to a list of tuples.
        topics_and_types = [x.items()[0] for x in param['topics']]

        for (param_topic_name, topic_type_str) in topics_and_types:
            if not rospy.has_param(param_topic_name):
                rospy.logerr('Expected parameter {}'.format(param_topic_name))
            rospy.loginfo('Getting all command maps that go on topic {}'.format(
                param_topic_name))
            command_mapping = rospy.get_param(param_topic_name)

            tmp = LanguageToMessageTranslator.parse_command_mapping(
                param_topic_name, topic_type_str, command_mapping)

            self._nl_command_map.update(tmp)

        rospy.loginfo('NL control running, listening to topic: {}.'.format(self.nl_command_topic))

    def print_command_map(self, show_token=False):
        rospy.loginfo("All available commands:")
        for (command, (topic, message)) in self._nl_command_map.iteritems():
            if show_token:
                rospy.loginfo('  {} -> {}'.format(command, message.data))
            else:
                rospy.loginfo('  {}'.format(command))
        rospy.loginfo('That is all.')

    @classmethod
    def parse_command_mapping(cls, topic_name, topic_type_str, command_mapping):
        """

        :param topic_name:
        :param topic_type_str:
        :param command_mapping: dict[basestring, Type] The mapping from
        natural language -> control token (expressed in whatever type).
        :return:

        This returns a map of:
        dict{command -> (topic, message)}
        where the command is the string to match, the topic is a string, and the
        message is the actual message type (ready to publish).
        """
        ret = {}

        msg_t = cls.string_to_type[topic_type_str]
        if not msg_t:
            rospy.logwarn('Message type {} is not know, skipping it'.format(
                topic_type_str))
            return ret  # Empty dict.

        # Make a command map for all commands, turning them into the correct
        # message type (e.g., std_msgs.String).
        for (nl_command, token) in command_mapping.items():
            token_msg = msg_t(token)  # Create an actual publishable message.
            ret[nl_command] = (topic_name, token_msg)

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

        if command in self._nl_command_map:
            rospy.loginfo('Heard command: {}'.format(command))
            (topic, message) = self._nl_command_map[command]
            rospy.loginfo('Sending command; topic[{}], message[{}]'.format(
                topic, message))

            # NOTE It may be useful to start all the publishers at startup.
            publisher = rospy.Publisher(topic, String, queue_size=1)
            publisher.publish(message)
        else:
            rospy.logwarn('Unknown NL command: {}'.format(command))


def run():
    rospy.init_node('message_control')

    translator = LanguageToMessageTranslator()

    translator.load_nl_command_map('allegro_hand_control')

    translator.print_command_map()

    rospy.spin()


if __name__ == '__main__':
    run()

__author__ = 'felixd'

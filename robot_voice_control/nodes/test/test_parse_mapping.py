#!/usr/bin/env python

import unittest

import rostest
import std_msgs
import genpy

from robot_voice_control.nodes.message_control import LanguageToMessageTranslator


class TestCase(unittest.TestCase):

    def setUp(self):
        self.setupParameters()
        pass

    def setupTranslator(self):
        self.translator = LanguageToMessageTranslator()

    def setupParameters(self):
        self.control_param_name = '/test_topic'

        # Since we cannot access the parameter server directly, fake it by
        # just storing the resulting parameter dictionary here. This is the same
        # as loading the test_params.yaml file and then calling
        # rospy.get_param(/test_topic)
        self.params = {'basic_topic':
                           {'input': 'output',
                            'input with spaces': 'output with spaces'},
                       'more': {'complicated': {'topic': {'input': 42}}},
                       'topics': [{'basic_topic': 'String'},
                                  {'more/complicated/topic': 'Int32'},
                                  {'/global/topic': 'Float32'},
                                  {'unknown_type': 'Unknown'}],
                       'unknown_topic': {'input': 'does not matter'},
                       'unknown_type': {'input': 'does not matter'}}

        # Convert a list of dictionaries to a dictionary.
        self.topics_and_types = dict([x.items()[0] for x in self.params['topics']])

        # This is a global topic, it is stored separately.
        self.global_param = {'input': 3.14159}

    def testSetup(self):
        self.setupTranslator()
        self.assertIsNotNone(self.translator)
        self.assertIsNotNone(self.translator.nl_command_map)

    def test_parse_command_map_wrong_type(self):
        not_a_type = 'NotAType'
        ret = LanguageToMessageTranslator.parse_command_mapping(
            'topic_name', not_a_type, {'a': 'b'})
        self.assertIsNotNone(ret)

    # Test string->msg type conversions.
    def test_string_to_type_String(self):
        ret = LanguageToMessageTranslator.string_to_type['String']
        self.assertEqual(std_msgs.msg.String, ret)

    def test_string_to_type_Float(self):
        ret = LanguageToMessageTranslator.string_to_type['Float32']
        self.assertEqual(std_msgs.msg.Float32, ret)

    def test_string_to_type_Int(self):
        ret = LanguageToMessageTranslator.string_to_type['Int32']
        self.assertEqual(std_msgs.msg.Int32, ret)

    def test_string_to_type_Other(self):
        ret = LanguageToMessageTranslator.string_to_type['NotAType']
        self.assertIsNone(ret)

    def test_setup_params(self):
        self.assertEqual(4, len(self.params['topics']))

        # Ensure both relative topics are *under* the params
        self.assertIn('basic_topic', self.params)
        self.assertEqual('String', self.params['topics'][0]['basic_topic'])
        self.assertIn('more', self.params)  # slashes become sub-dicts
        self.assertIn('unknown_type', self.params)
        self.assertIn('unknown_topic', self.params)

        # Basic params
        basic = self.params['basic_topic']
        self.assertEqual(2, len(basic))
        self.assertIn('input', basic)
        self.assertIn('input with spaces', basic)

    def test_topic_types_params(self):
        types_str = [x.values()[0] for x in self.params['topics']]

        types_real = [LanguageToMessageTranslator.string_to_type[x] for x in types_str]

        self.assertEqual(4, len(types_real))
        self.assertIn(None, types_real)
        self.assertIn(std_msgs.msg.Float32, types_real)
        self.assertIn(std_msgs.msg.Int32, types_real)
        self.assertIn(std_msgs.msg.String, types_real)

    def test_parse_command_map_basic(self):
        # Turn the commands defined in 'basic_topic' into a command mapping.
        topic = 'basic_topic'
        topic_type_str = self.topics_and_types[topic]
        commands = self.params[topic]
        ret = LanguageToMessageTranslator.parse_command_mapping(
            topic, topic_type_str, commands)

        self.assertIsNotNone(ret)

        # Should have the same number of elements as commands.
        self.assertEqual(len(commands), len(ret))

        for cmd in commands:  # All commands are in the map
            self.assertIn(cmd, ret)

            # Inspect each tuple: (topic, message)
            x = ret[cmd]
            self.assertEqual(2, len(x))
            self.assertEqual(topic, x[0])
            self.assertIsInstance(x[1], genpy.Message)
            self.assertIsInstance(x[1], std_msgs.msg.String)

        self.assertEqual('output', ret['input'][1].data)
        self.assertEqual('output with spaces', ret['input with spaces'][1].data)

    def test_parse_command_map_unknown_type(self):
        # Same as above, use an unknown message type
        topic = 'unknown_type'
        topic_type_str = self.topics_and_types[topic]
        commands = self.params[topic]
        ret = LanguageToMessageTranslator.parse_command_mapping(
            topic, topic_type_str, commands)

        self.assertIsNotNone(ret)
        self.assertEqual({}, ret)

    def test_parse_command_map_global_topic(self):
        topic = '/global/topic'
        topic_type_str = self.topics_and_types[topic]
        commands = self.global_param  # This is what would have been loaded.
        ret = LanguageToMessageTranslator.parse_command_mapping(
            topic, topic_type_str, commands)

        self.assertIsNotNone(ret)
        # Should have the same number of elements as commands.
        self.assertEqual(len(commands), len(ret))

        for cmd in commands:  # All commands are in the map
            self.assertIn(cmd, ret)

            # Inspect each tuple: (topic, message)
            x = ret[cmd]
            self.assertEqual(2, len(x))
            self.assertEqual(topic, x[0])
            self.assertIsInstance(x[1], genpy.Message)
            self.assertIsInstance(x[1], std_msgs.msg.Float32)

        self.assertEqual(3.14159, ret['input'][1].data)


if __name__ == '__main__':
    rostest.rosrun('robot_voice_control', 'test_parse_mapping', TestCase)

__author__ = 'felixd'

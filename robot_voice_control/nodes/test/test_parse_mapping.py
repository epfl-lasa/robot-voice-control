#!/usr/bin/env python

import unittest

import rospy
import rostest
import std_msgs
import rosparam

from robot_voice_control.nodes.message_control import LanguageToMessageTranslator


class TestCase(unittest.TestCase):

    def setUp(self):
        #
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

        # This is a global topic, it is stored separately.
        self.global_param = {'input': 3.14159}

    def testSetup(self):
        self.setupTranslator()
        self.assertIsNotNone(self.translator)

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
        self.setupParameters()

        self.assertEqual(4, len(self.params['topics']))

        # Ensure both relative topics are *under* the params
        self.assertIn('basic_topic', self.params)
        self.assertIn('more', self.params)  # slashes become sub-dicts
        self.assertIn('unknown_type', self.params)
        self.assertIn('unknown_topic', self.params)

        # Basic params
        basic = self.params['basic_topic']
        self.assertEqual(2, len(basic))
        self.assertIn('input', basic)
        self.assertIn('input with spaces', basic)

    def ztest_parse_command_map(self):
        self.fail('create an input map')
        self.fail('set the type')
        self.fail('set the topic name')
        self.fail('ensure the correct output')




if __name__ == '__main__':
    rostest.rosrun('robot_voice_control', 'test_parse_mapping', TestCase)

__author__ = 'felixd'

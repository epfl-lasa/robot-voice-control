#!/usr/bin/env python

import unittest
import rostest
import std_msgs

from robot_voice_control.nodes.message_control import LanguageToMessageTranslator


class TestCase(unittest.TestCase):

    def setUp(self):
        #
        pass

    def setupTranslator(self):
        self.translator = LanguageToMessageTranslator()

    def testSetup(self):
        self.setupTranslator()
        self.assertIsNotNone(self.translator)

    def test_parse_command_map_wrong_type(self):
        not_a_type = 'NotAType'
        ret = LanguageToMessageTranslator.parse_command_mapping(
            'topic_name', not_a_type, {})
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




if __name__ == '__main__':
    rostest.rosrun('robot_voice_control', 'test_parse_mapping', TestCase)

__author__ = 'felixd'

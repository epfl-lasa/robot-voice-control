#!/usr/bin/env python

import unittest
import rostest

from robot_voice_control.nodes.message_control import LanguageToMessageTranslator

class TestCase(unittest.TestCase):

    def setUp(self):
        self.translator = LanguageToMessageTranslator()

    def testSetup(self):
        self.assertIsNotNone(self.translator)

    def test_something(self):
        self.assertTrue(True)
        pass


if __name__ == '__main__':
    rostest.rosrun('test_parse_mapping', 'test_parse_mapping', TestCase)

__author__ = 'felixd'
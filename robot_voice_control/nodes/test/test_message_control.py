"""

Test cases for the entire pipeline.

"""
import unittest
from robot_voice_control.nodes.message_control import LanguageToMessageTranslator
from robot_voice_control.nodes.test.load_test_params import get_test_params


class TestCase(unittest.TestCase):

    def setUp(self):
        self.ltm = LanguageToMessageTranslator()
        self.params = get_test_params()


    def test_something(self):
        self.assertEqual(True, False)

if __name__ == '__main__':
    unittest.main()

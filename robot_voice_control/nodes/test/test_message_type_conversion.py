import unittest
from robot_voice_control.nodes.message_control import LanguageToMessageTranslator
import std_msgs


class TestCase(unittest.TestCase):

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

    def test_string_to_type_UInt(self):
        ret = LanguageToMessageTranslator.string_to_type['UInt32']
        self.assertEqual(std_msgs.msg.UInt32, ret)

    def test_string_to_type_Bool(self):
        ret = LanguageToMessageTranslator.string_to_type['Bool']
        self.assertEqual(std_msgs.msg.Bool, ret)

    def test_string_to_type_Char(self):
        ret = LanguageToMessageTranslator.string_to_type['Char']
        self.assertEqual(std_msgs.msg.Char, ret)

    def test_string_to_type_Other(self):
        ret = LanguageToMessageTranslator.string_to_type['NotAType']
        self.assertIsNone(ret)


if __name__ == '__main__':
    unittest.main()

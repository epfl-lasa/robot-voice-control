import unittest

from robot_voice_control.nodes.message_control import LanguageToMessageTranslator
from robot_voice_control.nodes.test.load_test_params import get_test_params

class TestCase(unittest.TestCase):

    def setUp(self):
        self.ltm = LanguageToMessageTranslator()
        self.params = get_test_params()

        self.mock_called = None

    def test_available_meta_commands(self):
        available = self.ltm.available_meta_commands
        self.assertIn('list', available)
        self.assertIn('quit', available)
        self.assertIn('pause', available)
        self.assertIn('resume', available)

    def test_enabled_meta_commands_empty(self):
        # No enabled commands - get nothing back.

        # Reset the LTM.
        self.ltm = LanguageToMessageTranslator()
        enabled = self.ltm.enabled_meta_commands
        self.assertIsNotNone(enabled)
        self.assertEqual(0, len(enabled))

    def test_enabled_meta_commands(self):
        enabled = self.ltm.enabled_meta_commands
        self.assertIsNotNone(enabled)
        self.assertEqual(2, len(enabled))

        # Known & enabled.
        self.assertIn('quit', enabled)
        self.assertIn('list', enabled)

        # Known but not enabled by user.
        self.assertNotIn('resume', enabled)
        self.assertNotIn('pause', enabled)

        # Unknown
        self.assertNotIn('unknown', enabled)

    def setup_mock_methods(self):
        self.ltm._meta_behavior['list'] = self.mock_list
        self.ltm._meta_behavior['quit'] = self.mock_quit

    def mock_list(self):
        self.mock_called = 'list'

    def mock_quit(self):
        self.mock_called = 'quit'

    def test_quit_gets_called(self):
        self.assertEqual(None, self.mock_called)

        # send command for quit

        self.assertEqual('quit', self.mock_called)

    def test_list_gets_called(self):
        self.assertEqual(None, self.mock_called)

        # send command for list

        self.assertEqual('list', self.mock_called)

    def test_no_meta_command(self):
        self.assertEqual(None, self.mock_called)

        # send command for something completely different

        self.assertEqual(None, self.mock_called)


if __name__ == '__main__':
    unittest.main()

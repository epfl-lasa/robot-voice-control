"""
Since we cannot access the parameter server directly in test code, fake it by
just storing the resulting parameter dictionary here.

This is the same as loading the test_params.yaml file into the param server and
then calling rospy.get_param(/test_topic)
"""


def get_test_params():
    return {'basic_topic': {'input': 'output',
                            'input with spaces': 'output with spaces'},
            'more': {'complicated': {'topic': {'input': 42}}},
            'not': {'a': {'global': {'topic': {'input': 3.14159}}}},
            'topics': [{'basic_topic': 'String'},
                       {'more/complicated/topic': 'Int32'},
                       {'not/a/global/topic': 'Float32'},
                       {'unknown_type': 'Unknown'}],
            'unknown_topic': {'input': 'does not matter'},
            'unknown_type': {'input': 'does not matter'}
            }

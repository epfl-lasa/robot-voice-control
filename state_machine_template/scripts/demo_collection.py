#! /usr/bin/env python

import rospy
import smach


class DelayState(smach.State):
    outcome_ok = 'okay'
    outcomes = [outcome_ok]

    def __init__(self, delay):
        super(DelayState, self).__init__(outcomes=DelayState.outcomes)
        self._delay = delay

    def execute(self, user_data):
        rospy.loginfo('Sleeping for {} seconds...'.format(self._delay))
        rospy.sleep(self._delay)
        rospy.loginfo('Finished sleeping')
        return DelayState.outcome_ok


class DemoCollectionMachine(smach.StateMachine):
    outcome_success = 'success'
    outcome_failure = 'failure'
    outcomes = [outcome_success, outcome_failure]

    def __init__(self):

        super(DemoCollectionMachine, self).__init__(
            outcomes=DemoCollectionMachine.outcomes)

        delay1_name = 'DELAY_1'
        delay1_state = DelayState(1)

        delay2_name = 'DELAY_2'
        delay2_state = DelayState(2)

        delay3_name = 'DELAY_3'
        delay3_state = DelayState(3)

        with self:
            self.add(delay1_name, delay1_state,
                     transitions={DelayState.outcome_ok: delay2_name})
            self.add(delay2_name, delay2_state,
                     transitions={DelayState.outcome_ok: delay3_name})
            self.add(delay3_name, delay3_state,
                     transitions={DelayState.outcome_ok: DemoCollectionMachine.outcome_success})


if __name__ == '__main__':

    import smach_ros
    rospy.init_node('interactive_demo')

    machine = DemoCollectionMachine()

    # Visualize the machine.
    machine_viz = smach_ros.IntrospectionServer(
        'smash_server', machine, '/SM_ROOT')
    machine_viz.start()

    outcome = machine.execute()

    rospy.loginfo('Outcome: {}'.format(outcome))

    machine_viz.stop()

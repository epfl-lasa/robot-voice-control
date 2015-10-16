#! /usr/bin/env python

import rospy
import smach
import smach_ros
import sys

from std_msgs.msg import String

from demo_collection import DemoCollectionMachine


class SayState(smach.State):
    # A state in the state machine can have multiple outcomes. The outcomes must
    # be unique names.

    outcome_success = 'success'
    outcomes = [outcome_success]

    def __init__(self, message):
        # The state initialization can store information.

        # Make sure to specify the outcomes here.
        super(SayState, self).__init__(outcomes=SayState.outcomes)

        self._message = message  # Store the message to say later.

    def execute(self, user_data):
        # The execute function gets called when the node is active. In this
        # case, it just prints a message, sleeps.
        print('--- {} ---'.format(self._message))
        rospy.sleep(2)

        # The execute function *must* return one of its defined outcomes. Here,
        # we only have one outcome (SayState.outcome_success) so return it.
        return SayState.outcome_success


class ReadyState(smach.State):

    # This state has two possible outcomes.
    outcome_ready = 'ready'
    outcome_finished = 'finished'
    outcomes = [outcome_ready, outcome_finished]

    def __init__(self):
        # Again, specify the outcomes.
        smach.State.__init__(self, outcomes=ReadyState.outcomes)

        # Subscribe to a topic, defined using a callback.
        topic = '/nl_command_parsed'
        rospy.Subscriber(topic, String, self.callback, queue_size=1)

        # Internal data.
        self._finished = False

    def execute(self, userdata):
        # This is called when the node is active. Currently it waits for the
        # user to press enter.

        rospy.loginfo('Executing ReadyState')
        raw_input('Press enter to be ready...')

        # There are two outcomes possible from this state; always return one of
        # them.
        if self._finished:
            rospy.loginfo('State machine is finished.')
            return ReadyState.outcome_finished
        else:
            rospy.loginfo('The show will go on')
            return ReadyState.outcome_ready

    def callback(self, data):
        # This is the callback for the subscribed topic.
        msg = data.data
        rospy.loginfo('Got message: {}'.format(msg))

        if 'quit' in msg or 'stop' in msg or 'done' in msg:
            self._finished = True


class UserInteraction(smach.StateMachine):

    # A state machine similarly has possible outcomes.
    outcome_success = 'success'
    outcome_failure = 'failure'
    outcomes = [outcome_success, outcome_failure]

    def __init__(self):
        super(UserInteraction, self).__init__(
            outcomes=UserInteraction.outcomes)

        # Create the states and give them names here. Each state (an instance of
        # the class) has an associated name (a string), used by the transitions.
        say_state = SayState('Hello, world')
        say_name = 'SAY_HELLO'

        ready_state = ReadyState()
        ready_name = 'READY'

        collect_name = 'COLLECT'
        collect_machine = DemoCollectionMachine()

        finished_state = SayState("I am finished")
        finished_name = 'SAY_FINISHED'


        # All states are now defined. Connect them.
        with self:
            # The first state added is the initial state.
            self.add(say_name, say_state,
                     transitions={SayState.outcome_success: ready_name})

            # For each state, all connections must be mapped to another
            # state. In this example, the ready outcome from ReadyState goes to
            # the collect node (identified by collect_name), and the finshed
            # outcome goes to the SAY_FINISHED node (again, identified by its
            # name). It's important to remember that all transitions are defined
            # by *strings*, not the underlying nodes.
            self.add(ready_name, ready_state,
                     transitions={ReadyState.outcome_ready: collect_name,
                                  ReadyState.outcome_finished: finished_name})

            # Here the connected state is actually a whole other
            # StateMachine. This is valid as long as its outcomes are properly
            # connected.
            self.add(collect_name, collect_machine,
                     transitions={DemoCollectionMachine.outcome_success: say_name,  # Go back to say_state
                                  DemoCollectionMachine.outcome_failure: UserInteraction.outcome_failure})
            self.add(finished_name, finished_state,
                     transitions={SayState.outcome_success: UserInteraction.outcome_success})
        pass


def run(arguments):
    rospy.init_node('interactive_demo')

    # Define the state machine here.
    machine = UserInteraction()

    # Visualize the machine.
    machine_viz = smach_ros.IntrospectionServer(
        'smash_server', machine, '/SM_ROOT')
    machine_viz.start()

    # Run it.
    outcome = machine.execute()

    rospy.loginfo('Outcome: {}'.format(outcome))

    machine_viz.stop()

if __name__ == '__main__':
    arguments = sys.argv[1:]  # argv[0] is the program name.
    run(arguments)

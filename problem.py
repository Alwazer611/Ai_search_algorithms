
class Problem(object):

    def __init__(self, initial_state, goal_state=None):
        self.initial_state = initial_state
        self.goal_state = goal_state

    def actions(self, state):

        # Returns actions available in state
        raise NotImplementedError

    def result(self, state, action):
        # Returns the transition-state that results from executing action instate.
        raise NotImplementedError

    def is_goal(self, state):
        if state == self.goal_state:
            return True
        else:
            return False

    def action_cost(self, state1, action, state2):

        return 1

    def h(self, node):

        return 0

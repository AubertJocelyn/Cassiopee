from swarm_rescue.nosAjouts.Submission.States.StateManager import StateManager, \
    PUT_WOUNDED_IN_CENTER_TIMER_INITIAL_VALUE


class StateManagementAbilitiesMixin:
    stateManager: StateManager
    def getPutWoundedInCenterTimer(self):
        return self.stateManager.PutWoundedInCenterTimer

    def decreasePutWoundedInCenterTimer(self):
        self.stateManager.PutWoundedInCenterTimer -= 1

    def resetPutWoundedInCenterTimer(self):
        self.stateManager.PutWoundedInCenterTimer = PUT_WOUNDED_IN_CENTER_TIMER_INITIAL_VALUE

    def buildState(self, state_name):
        return self.stateManager.states[state_name](self)

    def updateState(self):
        self.stateManager.state = self.stateManager.state.get_next_state()
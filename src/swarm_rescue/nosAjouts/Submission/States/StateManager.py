from swarm_rescue.nosAjouts.Submission.States.AimCenter import AimCenter
from swarm_rescue.nosAjouts.Submission.States.AimWounded import AimWounded
from swarm_rescue.nosAjouts.Submission.States.CheckIsCarryingWounded import CheckIsCarryingWounded
from swarm_rescue.nosAjouts.Submission.States.Explore import Explore
from swarm_rescue.nosAjouts.Submission.States.GoToCenter import GoToCenter
from swarm_rescue.nosAjouts.Submission.States.GoToWounded import GoToWounded
from swarm_rescue.nosAjouts.Submission.States.CatchWounded import CatchWounded
from swarm_rescue.nosAjouts.Submission.States.PutWoundedInCenter import PutWoundedInCenter

PUT_WOUNDED_IN_CENTER_TIMER_INITIAL_VALUE = 5
class StateManager:
    def __init__(self, drone):
        self.PutWoundedInCenterTimer = PUT_WOUNDED_IN_CENTER_TIMER_INITIAL_VALUE
        self.states = {
        "AimCenter": AimCenter,
        "AimWounded": AimWounded,
        "Explore": Explore,
        "GoToCenter": GoToCenter,
        "GoToWounded": GoToWounded,
        "GraspWounded": CatchWounded,
        "PutWoundedInCenter": PutWoundedInCenter,
        "CheckIsCarryingWounded": CheckIsCarryingWounded,
    }
        self.state = Explore(drone)


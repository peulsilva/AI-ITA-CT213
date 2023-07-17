import random
import math
from constants import *


class FiniteStateMachine(object):
    """
    A finite state machine.
    """
    def __init__(self, state):
        self.state = state

    def change_state(self, new_state):
        self.state = new_state

    def update(self, agent):
        self.state.check_transition(agent, self)
        self.state.execute(agent)


class State(object):
    """
    Abstract state class.
    """
    def __init__(self, state_name):
        """
        Creates a state.

        :param state_name: the name of the state.
        :type state_name: str
        """
        self.state_name = state_name

    def check_transition(self, agent, fsm):
        """
        Checks conditions and execute a state transition if needed.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent):
        """
        Executes the state logic.

        :param agent: the agent where this state is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")


class MoveForwardState(State):
    def __init__(self):
        super().__init__("MoveForward")
        self.time=0


    def check_transition(self, agent, state_machine):
        if self.time>=MOVE_FORWARD_TIME:
            state_machine.change_state(MoveInSpiralState())
        elif agent.get_bumper_state():
            state_machine.change_state(GoBackState())


    def execute(self, agent):
        agent.set_velocity(FORWARD_SPEED,0)
        self.time+=SAMPLE_TIME



class MoveInSpiralState(State):
    def __init__(self):
        super().__init__("MoveInSpiral")
        self.time=0
    
    def check_transition(self, agent, state_machine):
        if self.time>=MOVE_IN_SPIRAL_TIME:
            state_machine.change_state(MoveForwardState())
        elif agent.get_bumper_state():
            state_machine.change_state(GoBackState())

    def execute(self, agent):
        #velocity=omega*radius
        agent.set_velocity(FORWARD_SPEED,ANGULAR_SPEED/(INITIAL_RADIUS_SPIRAL+SPIRAL_FACTOR*self.time))
        self.time+=SAMPLE_TIME



class GoBackState(State):
    def __init__(self):
        super().__init__("GoBack")
        self.time=0

    def check_transition(self, agent, state_machine):
        if self.time>=GO_BACK_TIME:
            state_machine.change_state(RotateState())

    def execute(self, agent):
        agent.set_velocity(BACKWARD_SPEED,0)
        self.time+=SAMPLE_TIME


class RotateState(State):
    def __init__(self):
        super().__init__("Rotate")
        self.rotated=0
        self.random_angle=2*math.pi*random.random()

    def check_transition(self, agent, state_machine):
        if self.rotated>=self.random_angle:
            state_machine.change_state(MoveForwardState())
    
    def execute(self, agent):
        agent.set_velocity(0,ANGULAR_SPEED)
        self.rotated+=ANGULAR_SPEED*SAMPLE_TIME

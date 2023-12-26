from abc import ABC, abstractmethod
import numpy as np
from pyglet.window import key

from gym_duckietown.envs import DuckietownEnv


class MovementHandler(ABC):
    
    @abstractmethod
    def get_movement() -> np.ndarray:
        """
        Returns a movement vector of the form [v, omega]
        """
        pass



class KeyboardMovementHandler(MovementHandler):

    def __init__(self, env : DuckietownEnv):
        self.env : DuckietownEnv = env
        self.key_handler = key.KeyStateHandler()
        env.unwrapped.window.push_handlers(self.key_handler)

    def get_movement(self) -> np.ndarray:
        action = np.array([0.0, 0.0]) # Movement action

        if self.key_handler[key.UP]:
            action = np.array([0.44, 0.0])
        if self.key_handler[key.DOWN]:
            action = np.array([-0.44, 0])
        if self.key_handler[key.LEFT]:
            action = np.array([0.35, +1])
        if self.key_handler[key.RIGHT]:
            action = np.array([0.35, -1])
        if self.key_handler[key.SPACE]:
            action = np.array([0, 0])

        # Speed boost
        if self.key_handler[key.LSHIFT]:
            action *= 1.5

        return action
#!/usr/bin/env python
# manual

"""
This script allows you to manually control the simulator or Duckiebot
using the keyboard arrows.
"""

import sys
import argparse
from PIL import Image
import pyglet
from pyglet.window import key
import numpy as np
import gym
import gym_duckietown
from gym_duckietown.envs import DuckietownEnv
from gym_duckietown.wrappers import UndistortWrapper

from movement_handler import KeyboardMovementHandler, MovementHandler


def update(dt, movement_handler : MovementHandler, env : DuckietownEnv):
    """
    This function is called at every frame to handle
    movement/stepping and redrawing
    """
    action = movement_handler.get_movement() # Movement action
    obs, reward, done, info = env.step(action)
    im = Image.fromarray(obs) # Frame being captured
    
    # SLAM algorithm
    # TODO(marhcouto): add python ROS node to publish the image for the SLAM algorithm node

    env.render()



def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('--env-name', default=None)
    parser.add_argument('--map-name', default='udem1')
    parser.add_argument('--distortion', default=False, action='store_true')
    parser.add_argument('--draw-curve', action='store_true', help='draw the lane following curve')
    parser.add_argument('--draw-bbox', action='store_true', help='draw collision detection bounding boxes')
    parser.add_argument('--domain-rand', action='store_true', help='enable domain randomization')
    parser.add_argument('--frame-skip', default=1, type=int, help='number of frames to skip')
    parser.add_argument('--seed', default=1, type=int, help='seed')
    args = parser.parse_args()

    if args.env_name and args.env_name.find('Duckietown') != -1:
        env : DuckietownEnv = DuckietownEnv(
            seed = args.seed,
            map_name = args.map_name,
            draw_curve = args.draw_curve,
            draw_bbox = args.draw_bbox,
            domain_rand = args.domain_rand,
            frame_skip = args.frame_skip,
            distortion = args.distortion,
        )
    else:
        env : DuckietownEnv = gym.make(args.env_name)

    env.reset()
    env.render()

    @env.unwrapped.window.event
    def on_key_press(symbol, modifiers):
        """
        This handler processes keyboard commands that
        control the simulation
        """

        if symbol == key.BACKSPACE or symbol == key.SLASH:
            print('RESET')
            env.reset()
            env.render()
        elif symbol == key.PAGEUP:
            env.unwrapped.cam_angle[0] = 0
        elif symbol == key.ESCAPE:
            env.close()
            sys.exit(0)

    movement_handler : MovementHandler = KeyboardMovementHandler(env)
    pyglet.clock.schedule_interval(update, 1.0 / env.unwrapped.frame_rate, movement_handler, env)

    # Enter main event loop
    pyglet.app.run()

    env.close()


if __name__ == "__main__":
    main()
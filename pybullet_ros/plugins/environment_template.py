#!/usr/bin/env python3

from pybullet_ros.plugins.environment import Environment as DefaultEnv


class Environment(DefaultEnv):
    def __init__(self, pybullet, **kargs):
        super().__init__(pybullet)

    # override parent method
    def load_environment_via_code(self):
        """
        This method provides the possibility for the user to define an environment via python code
        """
        self.node.get_logger.warn('loading custom environment via code!')

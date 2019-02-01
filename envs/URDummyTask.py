import collections
from dm_control.rl import control
from dm_control.suite import base


class URDummyTask(base.Task):

    def __init__(self):

        super(URDummyTask, self).__init__(random=None)

    def initialize_episode(self, physics):
        pass

    def get_observation(self, physics):
        obs = collections.OrderedDict()
        obs['position'] = physics.position()
        obs['velocity'] = physics.velocity()

    def get_reward(self, physics):
        return 0
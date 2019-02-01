import os
import numpy as np
from dm_control import mujoco
from dm_control.rl import control
from dm_control import viewer
from envs import URDummyTask

PATH = os.path.dirname(os.path.realpath(__file__))
MODEL_PATH = PATH + '/models/data_collection.xml'

physics = mujoco.Physics.from_xml_path(MODEL_PATH)
env = control.Environment(physics=physics, task=URDummyTask())
action_spec = env.action_spec()

# Define a uniform random policy.
def random_policy(time_step):
  del time_step  # Unused.
  return np.random.uniform(low=action_spec.minimum,
                           high=action_spec.maximum,
                           size=action_spec.shape)

viewer.launch(env, policy=random_policy)


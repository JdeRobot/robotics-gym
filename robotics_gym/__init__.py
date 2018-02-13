import logging
import os
from gym.envs.registration import register
logger = logging.getLogger(__name__)
# Gazebo
# ----------------------------------------

# Turtlebot envs
register(
    id='Turtlebot2LaserEnv-v0',
    entry_point='robotics-gym.envs.gazebo.turtlebot2:Turtlebot2LaserEnv',
    # More arguments here
)

os.environ["GYM_ROBOTICS_TURTLEBOT_OCTA_GAZEBO_WORLD_FILE"] = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'envs/gazebo/assets/worlds/turtlebot2_octa.world')
os.environ["GYM_ROBOTICS_TURTLEBOT_RECT_GAZEBO_WORLD_FILE"] = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'envs/gazebo/assets/worlds/turtlebot2_rect.world')

register(
    id='WorldPlayerStageEnv-v0',
    entry_point='robotics-gym.envs.player_stage:WorldPlayerStageEnv',
    # More arguments here
)
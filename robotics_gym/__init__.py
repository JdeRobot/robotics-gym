import logging
import os
from gym.envs.registration import register
logger = logging.getLogger(__name__)

# Check ROS envrionment
if "ROS_MASTER_URI" not in os.environ:
    logger.warning("ROS_MASTER_URI not found. Have you initialize ROS environment?")

# Gazebo
# ----------------------------------------

# Turtlebot envs
register(
    id='Turtlebot2LaserEnv-v0',
    entry_point='robotics_gym.envs.gazebo.turtlebot:Turtlebot2LaserEnv',
    # More arguments here
)

register(
    id='Turtlebot3LaserEnv-v0',
    entry_point='robotics_gym.envs.gazebo.turtlebot:Turtlebot3LaserEnv',
    # More arguments here
)

register(
    id='Turtlebot2Laser2DEnv-v0',
    entry_point='robotics_gym.envs.gazebo.turtlebot:Turtlebot2Laser2DEnv',
    # More arguments here
)


os.environ["GYM_ROBOTICS_TURTLEBOT_OCTA_GAZEBO_WORLD_FILE"] = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'envs/gazebo/assets/worlds/turtlebot2_octa.world')
os.environ["GYM_ROBOTICS_TURTLEBOT_RECT_GAZEBO_WORLD_FILE"] = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'envs/gazebo/assets/worlds/turtlebot2_rect.world')

register(
    id='WorldPlayerStageEnv-v0',
    entry_point='robotics_gym.envs.player_stage:WorldPlayerStageEnv',
    # More arguments here
)

register(
    id='SimpleRoomsEnv-v0',
    entry_point='robotics_gym.envs.grid_worlds:SimpleRoomsEnv',
    # More arguments here
)

register(
    id='CliffWalkingEnv-v0',
    entry_point='robotics_gym.envs.grid_worlds:CliffWalkingEnv',
    # More arguments here
)

register(
    id='WindyGridworldEnv-v0',
    entry_point='robotics_gym.envs.grid_worlds:WindyGridworldEnv',
    # More arguments here
)
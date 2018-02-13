from robotics_gym.envs import player_stage_env
from robotics_gym.envs import specs

class WorldPlayerStageEnv(player_stage_env.PlayerStageEnv):

    supported = ['laser', 'ranger', 'camera', 'blobfinder']

    def __init__(self):

        if 'world' in specs.environment_specs:
            world = specs.environment_specs.world
        else:
            world = 'default'

        if 'port' in specs.environment_specs:
            port = specs.environment_specs.port
            player_stage_env.PlayerStageEnv.__init__(self, world, port)
        else:
            player_stage_env.PlayerStageEnv.__init__(self, world)

        print('PlayerStage init')


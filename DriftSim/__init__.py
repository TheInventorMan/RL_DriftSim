from gym.envs.registration import register

register(
    id='DriftSim-v0',
    entry_point='DriftSim.envs:DriftSim',
)

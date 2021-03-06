import math
import gym
from gym.utils import seeding
from gym import spaces
import numpy as np
from gym.envs.IARC.roombasim import environment
from gym.envs.IARC.roombasim import config as cfg
import pyglet
from baselines import logger
import skimage.measure
from gym.envs.IARC.IARC_Game_Board_Master import IARCEnv_Master


class IARCEnv_3(gym.Env, IARCEnv_Master):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 30
    }

    def __init__(self):
        self.action_space = spaces.Box(np.array([0.0, 0.0, 0., 0.]), np.array([1., 1., 1., 1.]), dtype=float)

        min_obs_template = list([0, 0, 0, False])
        max_obs_template = list([20, 20, math.pi * 2, True])
        min_obs = list()
        max_obs = list()
        for i in range(0, cfg.MISSION_NUM_TARGETS):  # (self.environment.roombas, start=1):
            min_obs = min_obs + min_obs_template
            max_obs = max_obs + max_obs_template

        min_obs = min_obs + list([0, 0])
        max_obs = max_obs + list([20, 20])

        self.observation_space = spaces.Box(np.asarray(min_obs), np.asarray(max_obs), dtype=float)

        self.init_Master()
        self.reset()

    def reset(self):
        self.reset_Master()

        self.state = list()
        for rmba in self.environment.roombas:
            if isinstance(rmba, environment.TargetRoomba):
                self.state = self.state + rmba.pos
                self.state = self.state + [rmba.heading]
                self.state = self.state + [(rmba.state == cfg.ROOMBA_STATE_FORWARD)]
        self.state = self.state + list(self.environment.agent.xy_pos)
        self.state = np.asarray(self.state)

        return self.state

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        rews = {'game_reward': 0.0, 'end_reward': 0.0, 'direction_reward': 0.0, 'targ_reward': 0.0, 'targ_reward2': 0.0}
        converge = False
        logger.logkv("targPos_x", action[0])
        logger.logkv("targPos_y", action[1])
        logger.dumpkvs()
        if action[0] > 1:
            rews['targ_reward2'] -= 0.01*(action[0] - 1)
        elif action[0] < 0:
            rews['targ_reward2'] += 0.01*action[0]
        if action[1] > 1:
            rews['targ_reward2'] -= 0.01*(action[1] - 1)
        elif action[1] < 0:
            rews['targ_reward2'] += 0.01*action[1]
        if action.ndim >= 2:
            action = action[0]
            action = np.clip(action, self.action_space.low, self.action_space.high)
            ac = {"aav_pos": action[0:2]*20.0, "ac_bool": bool(np.round(action[2])),
                  "top_or_front": bool(np.round(action[3]))}
        else:
            action = np.clip(action, self.action_space.low, self.action_space.high)
            ac = {"aav_pos": action[0:2]*20.0, "ac_bool": bool(np.round(action[2])),
                  "top_or_front": bool(np.round(action[3]))}
        # aav_targPos = self.environment.roombas[ac["rmba_sel"]].pos
        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))

        rmba_dists = dict()
        for rmba in self.environment.roombas:
            if isinstance(rmba, environment.TargetRoomba) and rmba.state is not cfg.ROOMBA_STATE_IDLE:
                rmba_dists[(np.linalg.norm(ac["aav_pos"] - rmba.pos))] = rmba

                # reward for moving in right direction
                rews["direction_reward"] += 10*self.getDirectionRew(rmba)
        # reward for targeting rmba
        rews["targ_reward"] -= 0.0001 * np.min(list(rmba_dists.keys()))
        ac["aav_pos"] = rmba_dists[np.min(list(rmba_dists.keys()))].pos

        def rmbaInteract():
            if ac["ac_bool"] and np.min(list(rmba_dists.keys())) < 0.35:
                rmba = rmba_dists[np.min(list(rmba_dists.keys()))]
                if ac["top_or_front"]:
                    rmba.collisions['top'] = True
                else:
                    rmba.collisions['front'] = True

        rews["game_reward"], rews["end_reward"], done = self._updateEnv(ac["aav_pos"], rmbaInteract)

        self.state = list()
        for rmba in self.environment.roombas:
            if isinstance(rmba, environment.TargetRoomba):
                self.state = self.state + rmba.pos
                self.state = self.state + [rmba.heading]
                self.state = self.state + [(rmba.state == cfg.ROOMBA_STATE_FORWARD)]
        self.state = self.state + list(self.environment.agent.xy_pos)

        info = {"time_ms": self.time_elapsed_ms, "rews": rews}
        #if self.time_elapsed_ms/1000 % 10 == 0:
         #   info = dict(info, **{"img": self._get_screen()})
        reward = 0
        for key, rew in rews.items():
            reward += rew
        return np.array(self.state), reward, done, info

    def _get_screen(self):
        if self.viewer is None:
            from gym.envs.IARC.roombasim.graphics import Display
            # from gym.envs.classic_control import rendering
            self.viewer = Display(self.environment, timescale=1.0, self_update=False)
            pyglet.app.event_loop.start()
        self.viewer.on_draw()#_roombas_only()
        buffer = pyglet.image.get_buffer_manager().get_color_buffer()
        image_data = buffer.get_image_data()
        arr = np.fromstring(image_data.data, dtype=np.uint8, sep='')
        arr = arr.reshape(buffer.height, buffer.width, 4)
        arr = arr[::-1, :, 0:3]
        arr1 = np.zeros([170, 170, 3], dtype=np.uint8)
        arr1[::, ::, 0] = skimage.measure.block_reduce(arr[::, ::, 0], (4, 4), np.max)
        arr1[::, ::, 1] = skimage.measure.block_reduce(arr[::, ::, 1], (4, 4), np.max)
        arr1[::, ::, 2] = skimage.measure.block_reduce(arr[::, ::, 2], (4, 4), np.max)

        return arr1

    def _render(self, mode='human', close=False):
        if close:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None
            return

        if self.viewer is None:
            from gym.envs.IARC.roombasim.graphics import Display
            # from gym.envs.classic_control import rendering
            self.viewer = Display(self.environment, timescale=1.0, self_update=False)
            pyglet.app.event_loop.start()

        # Redraw all windows
        self.viewer.update_time_only(0.1)
        for window in pyglet.app.windows:
            window.switch_to()
            window.dispatch_event('on_draw')
            window.flip()
            window._legacy_invalid = False
        pyglet.app.platform_event_loop.step(0.1)

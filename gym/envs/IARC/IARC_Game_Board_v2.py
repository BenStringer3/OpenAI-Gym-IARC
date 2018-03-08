import math
import gym
from gym.utils import seeding
from gym import spaces
import numpy as np

from gym.envs.IARC.roombasim import environment
# from gym.envs.IARC.roombasim.graphics import Display
from gym.envs.IARC.roombasim import config as cfg
import pyglet
import queue
from matplotlib import pyplot as plt

import skimage.measure
from collections import deque
import tensorflow as tf
# import roomba

NUM_OF_ACTIONS = 5



class IARCEnv_2(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 30
    }

    def __init__(self):
        self.viewer = None

        self.earlyTerminationTime_ms = None

        # self.action_space = spaces.Dict({"aav_pos" : spaces.Box(np.array([0, 0]), np.array([20, 20])), "exec" : spaces.MultiDiscrete([2, 2])})
        # self.action_space = spaces.Dict({"aav_pos" : spaces.Box(0, 20, (2,)), "exec" : spaces.MultiBinary(2)})
        self.action_space = spaces.MultiDiscrete([cfg.MISSION_NUM_TARGETS, 2, 2])


        import gym.envs.IARC.roombasim.pittras.config
        cfg.load(gym.envs.IARC.roombasim.pittras.config)

        self.environment = environment.Environment()
        self.environment.reset()

        # min_obs_template = np.array([0, 0, 0, False])
        # max_obs_template  = np.array([20, 20, math.pi*2, True])
        # min_obs = [min_obs_template]
        # max_obs = [max_obs_template]
        # for i in range(0, cfg.MISSION_NUM_TARGETS - 1): # (self.environment.roombas, start=1):
        #     min_obs = np.concatenate([min_obs, [min_obs_template]], axis=0)
        #     max_obs = np.concatenate([max_obs, [max_obs_template]], axis=0)
        # min_obs = np.expand_dims(min_obs, 2)
        # max_obs = np.expand_dims(max_obs, 2)

        min_obs_template = list([0, 0, 0, False])
        max_obs_template  = list([20, 20, math.pi*2, True])
        min_obs = list()
        max_obs = list()
        for i in range(0, cfg.MISSION_NUM_TARGETS): # (self.environment.roombas, start=1):
            min_obs = min_obs + min_obs_template
            max_obs = max_obs + max_obs_template

        min_obs  = min_obs + list([0, 0, 0])
        max_obs = max_obs + list([20, 20, cfg.MISSION_NUM_TARGETS - 1])

        self.observation_space = spaces.Box(np.asarray(min_obs), np.asarray(max_obs))

        # setup agent
        agent = cfg.AGENT([13, 10], 0)

        self.environment.agent = agent
        import time
        self._seed(round(time.time()))
        self._reset()

    def _reset(self):
        self.time_elapsed_ms = 0
        self.prev_score = 0
        self.environment.reset()

        # setup agent
        agent = cfg.AGENT([13, 10], 0)

        self.environment.agent = agent
        self.environment.agent.z_pos = 2


        # self.state = np.zeros((cfg.MISSION_NUM_TARGETS, 4))
        # i = 0
        # for rmba in self.environment.roombas:
        #     if isinstance(rmba, environment.TargetRoomba):
        #         self.state[i, 0] = rmba.pos[0]
        #         self.state[i, 1] = rmba.pos[1]
        #         self.state[i, 2] = rmba.heading
        #         self.state[i, 3] = (rmba.state == cfg.ROOMBA_STATE_FORWARD)
        #         i = i + 1
        # self.state = np.expand_dims(self.state, 2)
        self.last_rmba = 4
        self.last_converge = True
        self.state = list()
        for rmba in self.environment.roombas:
            if isinstance(rmba, environment.TargetRoomba):
                self.state = self.state + rmba.pos
                self.state = self.state + [rmba.heading]
                self.state = self.state + [(rmba.state == cfg.ROOMBA_STATE_FORWARD)]
        self.state = self.state + list(self.environment.agent.xy_pos) + list([self.last_rmba])
        self.state = np.asarray(self.state)

        return self.state

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]


    def _step(self, action):
        rews = {'game': 0.0, 'selection': 0.0, 'end' : 0.0, 'direction': 0.0}
        converge = False

        # assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))
        if action.ndim >= 2:
            action = action[0]
            ac = {"rmba_sel" : action[0], "ac_bool" : action[1], "top_or_front" : action[2]}
        else:
            ac = {"rmba_sel": action[0], "ac_bool": action[1], "top_or_front": action[2]}
        aav_targPos = self.environment.roombas[ac["rmba_sel"]].pos

        for rmba in self.environment.roombas:
            if isinstance(rmba, environment.TargetRoomba) and rmba.state is not cfg.ROOMBA_STATE_IDLE:
                rews["direction"] = (rmba.state == cfg.ROOMBA_STATE_FORWARD) * (
                        1 / math.pi / 3 * (math.fabs(math.pi - rmba.heading) - math.pi / 2)) / (
                                  cfg.MISSION_NUM_TARGETS - (
                                  self.environment.bad_exits + self.environment.good_exits))

        # ac["aav_pos"] = np.array([20.0, 0.0])
        dist2targ = np.linalg.norm(self.environment.agent.xy_pos - aav_targPos)
        if  dist2targ <= 0.35:

            self.environment.agent.xy_vel = np.array([0.0, 0.0])
            # rmba_dists = []
            # for i, rmba in enumerate(self.environment.roombas):
            #     if isinstance(rmba, environment.TargetRoomba) and rmba.state is not cfg.ROOMBA_STATE_IDLE:
            #         rmba_dists.append(np.linalg.norm(rmba.pos - ac["aav_pos"]))

            if ac["ac_bool"]:
                converge = True
                # print("hit a roomba")
                if ac["top_or_front"]:
                    self.environment.roombas[ac["rmba_sel"]].collisions['top'] = True
                else:
                    self.environment.roombas[ac["rmba_sel"]].collisions['front'] = True
        else:
            ang = math.atan2(aav_targPos[1] - self.environment.agent.xy_pos[1], aav_targPos[0] - self.environment.agent.xy_pos[0])
            self.environment.agent.xy_vel = np.array([cfg.DRONE_MAX_HORIZ_VELOCITY*np.cos(ang), cfg.DRONE_MAX_HORIZ_VELOCITY*np.sin(ang)])
        # if ac["ac_bool"]:
        #     self.environment.agent.xy_pos = self.environment.roombas[ac["rmba_sel"]].pos
        #     self.environment.agent.z_pos = 2
        #     if ac["top_or_front"]:
        #         self.environment.roombas[ac["rmba_sel"]].collisions['top'] = True
        #     else:
        #         self.environment.roombas[ac["rmba_sel"]].collisions['front'] = True

        done = False
        self.time_elapsed_ms += 100
        self.environment.update(0.1, self.time_elapsed_ms)
        rews["game"] += self.environment.score/100 - self.prev_score
        self.prev_score = self.environment.score/100
        # self.environment.agent.control(np.array([0.5, 0.5]), 0.001, 0.01)

        if action[0] != self.last_rmba and not self.last_converge:
            rews["selection"] -= 0.00002
        if action[0] == self.last_rmba and self.last_converge:
            rews["selection"] -= 0.00002

        self.last_rmba = action[0]
        self.last_converge = converge

        if (self.environment.bad_exits + self.environment.good_exits) >= cfg.MISSION_NUM_TARGETS:
            done = True
            rews["end"] += 11 * (10*60*1000 - self.environment.time_ms)/1000/60/10*self.environment.good_exits/cfg.MISSION_NUM_TARGETS
        if self.environment.time_ms >= 10*60*1000:
            # self.reset()
            done = True
        if self.earlyTerminationTime_ms is not None and self.earlyTerminationTime_ms <= self.environment.time_ms:
            done = True

        # self.q.pop()
        # self.q.appendleft(self._get_screen())
        # self.state = np.concatenate((self.q[0], self.q[self.q.maxlen - 1]), axis=1)

        self.state = list()
        for rmba in self.environment.roombas:
            if isinstance(rmba, environment.TargetRoomba):
                self.state = self.state + rmba.pos
                self.state = self.state + [rmba.heading]
                self.state = self.state + [(rmba.state == cfg.ROOMBA_STATE_FORWARD)]
        self.state = self.state + list(self.environment.agent.xy_pos) +list([self.last_rmba])
        # for rmba in self.environment.roombas:
        #     if isinstance(rmba, environment.ObstacleRoomba):
        #         self.state = self.state + rmba.pos

        info = {"time_ms": self.time_elapsed_ms, "converge": converge, "rews": rews}
        reward = 0
        for key, rew in rews.items():
            reward += rew
        return np.array(self.state), reward, done, info


    def _get_screen(self):
        self.viewer.on_draw_roombas_only()
        buffer = pyglet.image.get_buffer_manager().get_color_buffer()
        image_data = buffer.get_image_data()
        arr = np.fromstring(image_data.data, dtype=np.uint8, sep='')
        arr = arr.reshape(buffer.height, buffer.width, 4)
        arr = arr[::-1, :, 0:3]
        arr1 = np.zeros([170,170,3], dtype=np.uint8)
        arr1[::,::,0] = skimage.measure.block_reduce(arr[::,::,0], (4,4), np.max)
        arr1[::, ::, 1] = skimage.measure.block_reduce(arr[::, ::, 1], (4, 4), np.max)
        arr1[::, ::, 2] = skimage.measure.block_reduce(arr[::, ::, 2], (4, 4), np.max)

        # from matplotlib import pyplot as plt
        # plt.ion()
        # plt.imshow(arr1)
        # plt.show()
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

            # def update_func(delta, elapsed):
            #     self.environment.update(delta, elapsed)
            #
            # self.viewer.set_update_func(update_func)

            # pyglet.app.run()

            # pyglet.app.platform_event_loop.start()
            pyglet.app.event_loop.start()


        # timeout = pyglet.app.event_loop.idle()
        # self.viewer.update(0.1)
        # self.viewer.on_draw()
        # self.viewer.

        # dt = self.clock.update_time()
        # redraw_all = self.clock.call_scheduled_functions(dt)

        # Redraw all windows
        self.viewer.update_time_only(0.1)
        for window in pyglet.app.windows:
            window.switch_to()
            window.dispatch_event('on_draw')
            window.flip()
            window._legacy_invalid = False
        pyglet.app.platform_event_loop.step(0.1)
import math
import gym
from gym.utils import seeding
from gym import spaces
import numpy as np

from gym.envs.IARC.roombasim import environment
from gym.envs.IARC.roombasim.graphics import Display
from gym.envs.IARC.roombasim import config as cfg
import pyglet
import queue
from matplotlib import pyplot as plt

import skimage.measure
from collections import deque
import tensorflow as tf
# import roomba

NUM_OF_ACTIONS = 5



class IARCEnv_1(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 30
    }

    def __init__(self):
        self.viewer = None

        min_obs_template = list([0, 0, 0, False])
        max_obs_template  = list([20, 20, math.pi*2, True])

        # min_action = np.array([0.0, 0.0, 0.0, False, False, False])
        # max_action = np.array([20.0, 20.0, math.pi*2.0, True, True, True])
        # min_action = np.array([0.0, 0.0, -cfg.ROOMBA_LINEAR_SPEED, -cfg.ROOMBA_LINEAR_SPEED, False, False])
        # max_action = np.array([20.0, 20.0, cfg.ROOMBA_LINEAR_SPEED, cfg.ROOMBA_LINEAR_SPEED, True, True])
        # self.action_space = spaces.Box(min_action, max_action) # , shape=(1,))
        # self.action_space = spaces.MultiDiscrete((cfg.MISSION_NUM_TARGETS + 2)
        # self.action_space = spaces.MultiDiscrete([[0, cfg.MISSION_NUM_TARGETS - 1], [0, 1], [0, 1]])
        self.action_space = spaces.MultiDiscrete([cfg.MISSION_NUM_TARGETS, 2, 2])



        import gym.envs.IARC.roombasim.pittras.config
        cfg.load(gym.envs.IARC.roombasim.pittras.config)

        self.environment = environment.Environment()
        self.environment.reset()

        min_obs = list()
        max_obs = list()
        for rmba in self.environment.roombas:
            if isinstance(rmba, environment.TargetRoomba):
                min_obs = min_obs + min_obs_template
                max_obs = max_obs + max_obs_template
        # for rmba in self.environment.roombas:
        #     if isinstance(rmba, environment.ObstacleRoomba):
        #         min_obs = min_obs + list([0, 0])
        #         max_obs = max_obs + list([20, 20])
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
        #self._render()

        # for i in range(self.q.maxlen):
        #     self.q.append( self._get_screen())
        #
        # self.state = np.concatenate((self.q[0], self.q[self.q.maxlen - 1]), axis=1)


        self.state = list()
        for rmba in self.environment.roombas:
            if isinstance(rmba, environment.TargetRoomba):
                self.state = self.state + rmba.pos
                self.state = self.state + [rmba.heading]
                self.state = self.state + [(rmba.state == cfg.ROOMBA_STATE_FORWARD)]
        # for rmba in self.environment.roombas:
        #     if isinstance(rmba, environment.ObstacleRoomba):
        #         self.state = self.state + rmba.pos
        return np.array(self.state)

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # def grid2Pixels(self, gridVect):
    #     x = gridVect[0]
    #     y = gridVect[1]
    #     pixelVect = np.zeros((2, 1))
    #     pixelVect[0] = (x + self.grid_width / 2) * self.scale[0]
    #     pixelVect[1] = (y + self.grid_height / 2) * self.scale[1]
    #     return pixelVect
    #
    # def distance(self, obj1, obj2):
    #     return math.sqrt(math.pow(obj1[0] - obj2[0], 2) + math.pow(obj1[1] - obj2[1], 2))


    def _step(self, action):
        reward = 0
        # action = (np.tanh(action) + 1) / 2 * (self.action_space.high - self.action_space.low) + self.action_space.low
        # # action[3] = np.round(action[3])
        # action[4] = np.round(action[4])
        # action[5] = np.round(action[5])

        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))
        ac = {"rmba_sel" : action[0], "ac_bool" : action[1], "top_or_front" : action[2]}
        dist_dict = {"min_dist_all": None, "min_dist_ac" : None}
        distances = [None] * (cfg.MISSION_NUM_TARGETS)  # - (self.environment.bad_exits + self.environment.good_exits))
        i = 0
        for rmba in self.environment.roombas:
            if isinstance(rmba, environment.TargetRoomba):
                if rmba.state is cfg.ROOMBA_STATE_IDLE:
                    distances[i] = math.inf
                else:
                    reward += (rmba.state == cfg.ROOMBA_STATE_FORWARD) * (
                                1 / math.pi / 30 * (math.fabs(math.pi - rmba.heading) - math.pi / 2)) / (
                                          cfg.MISSION_NUM_TARGETS - (
                                              self.environment.bad_exits + self.environment.good_exits))
                    x_vel = np.cos(rmba.heading) * cfg.ROOMBA_LINEAR_SPEED * (rmba.state == cfg.ROOMBA_STATE_FORWARD)
                    y_vel = np.sin(rmba.heading) * cfg.ROOMBA_LINEAR_SPEED * (rmba.state == cfg.ROOMBA_STATE_FORWARD)
                    # distances[i] = np.sqrt(
                    #     np.power(action[0] - rmba.pos[0], 2) +
                    #     np.power(action[1] - rmba.pos[1], 2) +
                    #     np.power(self.action_space.high[0] / self.action_space.high[2] * (action[2] - x_vel), 2) +
                    #     np.power(self.action_space.high[0] / self.action_space.high[2] * (action[3] - y_vel), 2))
                    # distances[i] = np.sqrt(
                    #     np.power(action[0] - rmba.pos[0], 2) +
                    #     np.power(action[1] - rmba.pos[1], 2) +
                    #     np.power(self.action_space.high[0] / self.action_space.high[2] * (math.pi-math.fabs(math.fabs(action[2] - rmba.heading) - math.pi)), 2) +
                    #     np.power(self.action_space.high[0] / self.action_space.high[2] * (np.round(action[3]) - (rmba.state == cfg.ROOMBA_STATE_FORWARD)), 2))
                i += 1
        # dist_dict["min_dist_all"] = min(distances)
        if ac["ac_bool"]:
            from operator import itemgetter
            # dist_dict["min_dist_ac"] = min(distances)
            # target_iter =min(enumerate(distances), key=itemgetter(1))[0]
            self.environment.agent.xy_pos = self.environment.roombas[ac["rmba_sel"]].pos
            self.environment.agent.z_pos = 2
            if ac["top_or_front"]:
                self.environment.roombas[ac["rmba_sel"]].collisions['top'] = True
            else:
                self.environment.roombas[ac["rmba_sel"]].collisions['front'] = True

        done = False
        self.time_elapsed_ms += 100
        self.environment.update(0.1, self.time_elapsed_ms)
        reward += self.environment.score/1000 - self.prev_score
        self.prev_score = self.environment.score/1000
        # self.environment.agent.control(np.array([0.5, 0.5]), 0.001, 0.01)



        if (self.environment.bad_exits + self.environment.good_exits) >= cfg.MISSION_NUM_TARGETS:
            done = True
        if self.environment.time_ms >= 0.5*60*1000:
            # self.reset()
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
        # for rmba in self.environment.roombas:
        #     if isinstance(rmba, environment.ObstacleRoomba):
        #         self.state = self.state + rmba.pos

        return np.array(self.state), reward, done, dist_dict


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
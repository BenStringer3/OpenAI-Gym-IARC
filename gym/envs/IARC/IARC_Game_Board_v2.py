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
# from Redbird_AI.modelEnv import ModelEnv


class IARCEnv_2(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 30
    }

    def __init__(self):




        self.viewer = None

        self.earlyTerminationTime_ms = None
        self.render_bool = False

        self.action_space = spaces.MultiDiscrete([cfg.MISSION_NUM_TARGETS, 2, 2])


        import gym.envs.IARC.roombasim.pittras.config
        cfg.load(gym.envs.IARC.roombasim.pittras.config)

        self.environment = environment.Environment()
        self.environment.reset()

        min_obs_template = list([0, 0, 0, False])
        max_obs_template  = list([20, 20, math.pi*2, True])
        min_obs = list()
        max_obs = list()
        for i in range(0, cfg.MISSION_NUM_TARGETS): # (self.environment.roombas, start=1):
            min_obs = min_obs + min_obs_template
            max_obs = max_obs + max_obs_template

        min_obs  = min_obs + list([0, 0, 0])
        max_obs = max_obs + list([20, 20, cfg.MISSION_NUM_TARGETS - 1])
        self.last_rmba = 4

        self.observation_space = spaces.Box(np.asarray(min_obs), np.asarray(max_obs), dtype=np.float32)

        # setup agent
        agent = cfg.AGENT([13, 10], 0)
        self.environment.agent = agent

        self.reset()

        #TODO remove temporary test stuf
        self.test_set = False
        self.test_set2 = False
        arr = np.zeros([32, 32, 1], dtype=np.uint8)
        arr[5, 5, :] = 64
        arr[6, 31, :] = 64
        arr[31, 23, :] = 64
        arr[10, 9, :] = 64
        arr[0, 21, :] = 64
        arr[5, 15, :] = 64
        arr[9, 9, :] = 64
        arr[5, 16, :] = 64
        arr[15, 27, :] = 64
        arr[30, 9, :] = 64
        self.test_img = arr
        self.test_img2 = arr
        self.test_ob = self.observation_space.sample()
        self.test_ob2 = self.test_ob
        # self.modelEnv = ModelEnv(self.observation_space, gym.spaces.Box(0, 255, [64, 64])) # TODO remove shape hardcode

    def reset(self):
        self.time_elapsed_ms = 0
        self.prev_score = 0
        self.environment.reset()

        # setup agent
        agent = cfg.AGENT([13, 10], 0)

        self.environment.agent = agent
        self.environment.agent.z_pos = 2

        self.state = list()
        for rmba in self.environment.roombas:
            if isinstance(rmba, environment.TargetRoomba):
                self.state = self.state + rmba.pos
                self.state = self.state + [rmba.heading]
                self.state = self.state + [(rmba.state == cfg.ROOMBA_STATE_FORWARD)]
        self.state = self.state + list(self.environment.agent.xy_pos) + list([self.last_rmba])
        self.state = np.asarray(self.state)


        return self.state

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        # self.modelEnv.seed(self.np_random)
        return [seed]


    def step(self, action):
        rews = {'game_reward': 0.0, 'speed_reward' : 0.0, 'direction_reward': 0.0}

        # assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))
        if action.ndim >= 2:
            action = action[0]
            ac = {"rmba_sel" : action[0], "ac_bool" : action[1], "top_or_front" : action[2]}
        else:
            ac = {"rmba_sel": action[0], "ac_bool": action[1], "top_or_front": action[2]}
        aav_targPos = self.environment.roombas[ac["rmba_sel"]].pos

        for rmba in self.environment.roombas:
            if isinstance(rmba, environment.TargetRoomba) and rmba.state is not cfg.ROOMBA_STATE_IDLE:
                rews["direction_reward"] = (rmba.state == cfg.ROOMBA_STATE_FORWARD) * (
                        1 / math.pi / 3 * (math.fabs(math.pi - rmba.heading) - math.pi / 2)) / (
                                  cfg.MISSION_NUM_TARGETS - (
                                  self.environment.bad_exits + self.environment.good_exits))
        break_early = False
        for i in range(10):
            # ac["aav_pos"] = np.array([20.0, 0.0])
            dist2targ = np.linalg.norm(self.environment.agent.xy_pos - aav_targPos)
            if  dist2targ <= 0.35:
                self.environment.agent.xy_vel = np.array([0.0, 0.0])
                if ac["ac_bool"]:
                    # converge = True
                    # print("hit a roomba")
                    if ac["top_or_front"]:
                        self.environment.roombas[ac["rmba_sel"]].collisions['top'] = True
                    else:
                        self.environment.roombas[ac["rmba_sel"]].collisions['front'] = True
                    break_early = True
            else:
                ang = math.atan2(aav_targPos[1] - self.environment.agent.xy_pos[1], aav_targPos[0] - self.environment.agent.xy_pos[0])
                self.environment.agent.xy_vel = np.array([cfg.DRONE_MAX_HORIZ_VELOCITY*np.cos(ang), cfg.DRONE_MAX_HORIZ_VELOCITY*np.sin(ang)])

            self.time_elapsed_ms += 100
            self.environment.update(0.1, self.time_elapsed_ms)
            if self.render_bool:
                self._render()
            # if break_early:
            #     continue
        rews["game_reward"] += self.environment.score/100 - self.prev_score
        self.prev_score = self.environment.score/100
        rews["speed_reward"] += rews["game_reward"] * 0.1*(10*60*1000 - self.environment.time_ms)/1000/60/10


        done = False
        if (self.environment.bad_exits + self.environment.good_exits) >= cfg.MISSION_NUM_TARGETS:
            done = True
            # rews["end_reward"] += 11 * (10*60*1000 - self.environment.time_ms)/1000/60/10*self.environment.good_exits/cfg.MISSION_NUM_TARGETS
        if self.environment.time_ms >= 10*60*1000:
            # self.reset()
            done = True
        if self.earlyTerminationTime_ms is not None and self.earlyTerminationTime_ms <= self.environment.time_ms:
            done = True

        self.state = list()
        for rmba in self.environment.roombas:
            if isinstance(rmba, environment.TargetRoomba):
                self.state = self.state + rmba.pos
                self.state = self.state + [rmba.heading]
                self.state = self.state + [(rmba.state == cfg.ROOMBA_STATE_FORWARD)]
        self.state = self.state + list(self.environment.agent.xy_pos) +list([self.last_rmba])

        # img_true = self.get_screen()
        # self.modelEnv.step2(np.array(self.state), img_true)
        if self.environment.time_ms == 25000 and self.test_set is False:
            self.test_ob = np.array(self.state)
            self.test_img = self.get_screen2()
            self.test_set = True
        if self.environment.time_ms == 45000 and self.test_set2 is False:
            self.test_ob2 = np.array(self.state)
            self.test_img2 = self.get_screen2()
            self.test_set2 = True
        if np.random.random_integers(1, 5) % 2 == 0:
            test_ob = self.test_ob
            test_img = self.test_img
        else:
            test_ob = self.test_ob2
            test_img = self.test_img2

        info = {"time_ms": self.time_elapsed_ms, "rews": rews, "img": self.get_screen2(), "test_ob": test_ob}
        reward = 0
        for key, rew in rews.items():
            reward += rew
        return np.array(self.state), reward, done, info


    def get_screen(self):
        if self.viewer is None:
            from gym.envs.IARC.roombasim.graphics import Display
            # from gym.envs.classic_control import rendering
            self.viewer = Display(self.environment, timescale=1.0, self_update=False)
            pyglet.app.event_loop.start()
        self.viewer.on_draw_roombas_only()
        buffer = pyglet.image.get_buffer_manager().get_color_buffer()
        image_data = buffer.get_image_data()
        arr = np.fromstring(image_data.data, dtype=np.uint8, sep='')
        arr = arr.reshape(buffer.height, buffer.width, 4)
        arr = arr[::-1, :, 0:3]
        arr1 = np.zeros([170,170,3], dtype=np.uint8)
        arr1[::, ::, 0] = skimage.measure.block_reduce(arr[::,::,0], (4,4), np.max)
        arr1[::, ::, 1] = skimage.measure.block_reduce(arr[::, ::, 1], (4, 4), np.max)
        arr1[::, ::, 2] = skimage.measure.block_reduce(arr[::, ::, 2], (4, 4), np.max)

        # from matplotlib import pyplot as plt
        # plt.ion()
        # plt.imshow(arr1/64.0)
        # plt.show()

        # from PIL import Image
        # img = Image.fromarray(arr, 'RGB')
        # img.save('my.png')
        # img.show()
        return arr

    def get_screen2(self):
        img_size = 32 #TODO remove hardcode
        arr = np.zeros([img_size, img_size, 1], dtype=np.uint8)
        for rmba in self.environment.roombas:
            if isinstance(rmba, environment.TargetRoomba) and rmba.state is not cfg.ROOMBA_STATE_IDLE:
                arr[int(rmba.pos[0]/20.0*(img_size - 1)), int(rmba.pos[1]/20.0*(img_size - 1)), :] = 64
        return arr

    def get_example_img(self):
        img_size = 32 #TODO remove hardcode
        self.get_example_img.counter += 1
        if self.get_example_img.counter >= 25:
            arr = self.get_screen()

        else:
            arr = np.zeros([img_size, img_size, 1], dtype=np.uint8)
            arr[5, 5, :] = 64
            arr[6, 31, :] = 64
            arr[31, 23, :] = 64
            arr[10, 9, :] = 64
            arr[0, 21, :] = 64
            arr[5, 15, :] = 64
            arr[9, 9, :] = 64
            arr[5, 16, :] = 64
            arr[15, 27, :] = 64
            arr[30, 9, :] = 64
        #
        # arr1 = np.zeros([img_size, img_size, 1], dtype=np.uint8)
        # arr1[7, 5, :] = 64
        # arr1[6, 10, :] = 64
        # arr1[15, 23, :] = 64
        # arr1[10, 30, :] = 64
        # arr1[10, 21, :] = 64
        # arr1[5, 17, :] = 64
        # arr1[25, 9, :] = 64
        # arr1[5, 7, :] = 64
        # arr1[5, 27, :] = 64
        # arr1[30, 3, :] = 64

        return arr
    get_example_img.counter = 0

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
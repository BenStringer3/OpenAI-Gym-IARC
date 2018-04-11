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
from gym.envs.IARC.IARC_Game_Board_Master import IARCEnv_Master


class IARCEnv_4(gym.Env, IARCEnv_Master):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 30
    }

    def __init__(self):
        self.last_rmba=4
        self.action_space = spaces.MultiDiscrete([cfg.MISSION_NUM_TARGETS, 2, 2])

        min_obs_template = list([0, 0, 0, False])
        max_obs_template = list([20, 20, math.pi * 2, True])
        min_obs = list()
        max_obs = list()
        for i in range(0, cfg.MISSION_NUM_TARGETS):  # (self.environment.roombas, start=1):
            min_obs = min_obs + min_obs_template
            max_obs = max_obs + max_obs_template

        min_obs = min_obs + list([0, 0, 0])
        max_obs = max_obs + list([20, 20, 9])

        self.observation_space = spaces.Box(np.asarray(min_obs), np.asarray(max_obs), dtype=np.float32)
        # self.observation_space = spaces.Tuple([spaces.Box(np.array([0, 0, 0, False]), np.array([20, 20, math.pi*2, True]), dtype=np.float32) for _ in range(cfg.MISSION_NUM_TARGETS)])

        self.init_Master()
        self.reset()

        self.num_test_imgs = 10
        self.numTestsActivated = 0
        # TODO remove temporary test stuf
        self.test_set = [False] * self.num_test_imgs
        self.arr = np.zeros([32, 32, 1], dtype=np.uint8)
        self.arr[5, 5, :] = 255
        self.arr[6, 31, :] = 255
        self.arr[31, 23, :] = 255
        self.arr[10, 9, :] = 255
        self.arr[0, 21, :] = 255
        self.arr[5, 15, :] = 255
        self.arr[9, 9, :] = 255
        self.arr[5, 16, :] = 255
        self.arr[15, 27, :] = 255
        self.arr[30, 9, :] = 255
        self.test_img = [self.arr] * self.num_test_imgs
        self.test_img_triggerTime = [0.0] * self.num_test_imgs
        self.test_ob = [self.observation_space.sample()] * self.num_test_imgs
        self.test_fov_mask = [self.observation_space.sample()] * self.num_test_imgs
        for i in range(self.num_test_imgs):
            self.test_img_triggerTime[i] = 2000 + i * 58000

    def _get_state(self):
        state = list()
        fov_mask = list()
        for rmba in self.environment.roombas:
            if isinstance(rmba, environment.TargetRoomba):
                if rmba.state == cfg.ROOMBA_STATE_IDLE:
                    fov_mask = fov_mask + [np.nan, np.nan, np.nan, np.nan]
                else:
                    fov_mask = fov_mask + [1, 1, 1, 1]
                state = state + rmba.pos
                state = state + [rmba.heading]
                state = state + [(rmba.state == cfg.ROOMBA_STATE_FORWARD)]
        state = state + list(self.environment.agent.xy_pos)+list([self.last_rmba])
        fov_mask = fov_mask + [1, 1, 1]
        state = np.array(state, dtype=np.float32)
        fov_mask = np.array(fov_mask, dtype=np.float32)
        return state/self.observation_space.high, fov_mask

    def reset(self):
        self.reset_Master()
        self.state, _ = self._get_state()
        self.last_good_exits = 0
        self.last_bad_exits = 0
        return self.state


    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        np.random.seed(seed)
        return [seed]

    def step(self, action):
        rews = {'game_reward': 0.0, 'speed_reward': 0.0, 'direction_reward': 0.0, 'target_reward': 0.0, 'num_good_exits':0, 'num_bad_exits':0}


        # assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))
        if action.ndim >= 2:
            action = action[0]
            ac = {"rmba_sel": action[0], "ac_bool": action[1], "top_or_front": action[2]}
        else:
            ac = {"rmba_sel": action[0], "ac_bool": action[1], "top_or_front": action[2]}
        aav_targPos = self.environment.roombas[ac["rmba_sel"]].pos

        if self.environment.roombas[ac["rmba_sel"]].state is cfg.ROOMBA_STATE_IDLE:
            rews["target_reward"] -= 2*self.numRenderSecs

        for rmba in self.environment.roombas:
            if isinstance(rmba, environment.TargetRoomba) and rmba.state is not cfg.ROOMBA_STATE_IDLE:
                rews["direction_reward"] += 20*self.numRenderSecs*self.getDirectionRew(rmba)
                # rmba_dists[(np.linalg.norm(ac["aav_pos"] - rmba.pos))] = rmba


        def rmbaInteract():
            if ac["ac_bool"]:
                if ac["top_or_front"]:
                    self.environment.roombas[ac["rmba_sel"]].collisions['top'] = True
                else:
                    self.environment.roombas[ac["rmba_sel"]].collisions['front'] = True

        rews["game_reward"], rews["speed_reward"], done = self._updateEnv(aav_targPos, rmbaInteract)
        rews['num_good_exits'] = self.environment.good_exits - self.last_good_exits
        rews['num_bad_exits'] = self.environment.bad_exits - self.last_bad_exits
        self.last_good_exits = self.environment.good_exits
        self.last_bad_exits = self.environment.bad_exits

        self.state, fov_mask = self._get_state()


        for i in range(self.num_test_imgs):
            if self.environment.time_ms == self.test_img_triggerTime[i] and self.test_set[i] is False:
                self.test_ob[i] = self.state
                self.test_img[i] = self.get_screen()
                self.test_set[i] = True
                self.test_fov_mask[i] = fov_mask
                self.numTestsActivated += 1
        if self.numTestsActivated == 1:
            test_num = 0
        else:
            test_num = np.random.random_integers(0, self.numTestsActivated - 1)
        test_ob = self.test_ob[test_num]
        test_img = self.test_img[test_num]
        test_fov_mask = self.test_fov_mask[test_num]
        if np.array_equal(test_img, self.arr):
            print("uh-oh spagettios")

        info = {"time_ms": self.time_elapsed_ms, "rews": rews, "test_ob": test_ob,
                "test_img": test_img,  "test_fov_mask": test_fov_mask, "fov_mask":fov_mask}
        if done:
            self.environment.reset()
            info["img"] = self.get_screen()
        else:
            info["img"] = self.get_screen()
        reward = 0
        rews["num_bad_exits"] *= -1
        for key, rew in rews.items():
            reward += rew
        return self.state, reward, done, info

    # def get_screen2(self):
    #     if self.viewer is None:
    #         from gym.envs.IARC.roombasim.graphics import Display
    #         # from gym.envs.classic_control import rendering
    #         self.viewer = Display(self.environment, timescale=1.0, self_update=False)
    #         pyglet.app.event_loop.start()
    #     self.viewer.on_draw_roombas_only()
    #     buffer = pyglet.image.get_buffer_manager().get_color_buffer()
    #     image_data = buffer.get_image_data()
    #     arr = np.fromstring(image_data.data, dtype=np.uint8, sep='')
    #     arr = arr.reshape(buffer.height, buffer.width, 4)
    #     arr = arr[::-1, :, 0:3]
    #     arr1 = np.zeros([170, 170, 3], dtype=np.uint8)
    #     arr1[::, ::, 0] = skimage.measure.block_reduce(arr[::, ::, 0], (4, 4), np.max)
    #     arr1[::, ::, 1] = skimage.measure.block_reduce(arr[::, ::, 1], (4, 4), np.max)
    #     arr1[::, ::, 2] = skimage.measure.block_reduce(arr[::, ::, 2], (4, 4), np.max)
    #
    #     # from matplotlib import pyplot as plt
    #     # plt.ion()
    #     # plt.imshow(arr1/64.0)
    #     # plt.show()
    #
    #     # from PIL import Image
    #     # img = Image.fromarray(arr, 'RGB')
    #     # img.save('my.png')
    #     # img.show()
    #     return arr

    def get_screen(self):
        img_size = 32  # TODO remove hardcode
        arr = np.zeros([img_size, img_size, 1], dtype=np.uint8)
        for rmba in self.environment.roombas:
            if isinstance(rmba, environment.TargetRoomba) and rmba.state is not cfg.ROOMBA_STATE_IDLE:
                arr[int(rmba.pos[0] / 20.0 * (img_size - 1)), int(rmba.pos[1] / 20.0 * (img_size - 1)), :] = 255
        return arr

    def get_example_img(self):
        img_size = 32  # TODO remove hardcode
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
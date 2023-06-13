from gym import spaces
import numpy as np
from .pybullet_evo.gym_locomotion_envs import HalfCheetahBulletEnv
import copy
from utils import BestEpisodesVideoRecorder
# import h5py
import os
import gc

class HalfCheetahEnv(object):
    def __init__(self, config = {'env' : {'render' : True, 'record_video': False}}):
        self._config = config
        self._render = self._config['env']['render']
        self._record_video = self._config['env']['record_video']
        # CRAS14 Memos: Originally 6 number of modifiable variables, depending on the parts of the legs that will be changing
        self._current_design = [1.0] * 4
        self._config_numpy = np.array(self._current_design)
        self.design_params_bounds = [(0.8, 2.0)] * 4 # CRAS14 Memos: limiter for designs
        self._env = HalfCheetahBulletEnv(render=self._render, design=self._current_design)

        # first five designs that will be changed and there are "5 robots"
        """
        # initial parameters
            [1.0] * 6,
            [1.41, 0.96, 1.97, 1.73, 1.97, 1.17],
            [1.52, 1.07, 1.11, 1.97, 1.51, 0.99],
            [1.08, 1.18, 1.39, 1.76 , 1.85, 0.92],
            [0.85, 1.54, 0.97, 1.38, 1.10, 1.49],
        """
        
        # CRAS14
        # The initial simulation parameters for the environment
        # These are the parameters that are optimized
        # They are just coefficients for the inital values of the modifiable legs

        # CRAS14 Because we only have one idea
        # parameters depends on the amount of modifiable values
        # depending on the parts of the legs that will be changing
        # or it can be the self._current_design
        self.init_sim_params = [
            [1.0]*4,
            [2.0]*4,
            [3.0]*4
        ]
        # CRAS14 matrix multiplication error caused by the observation and action space
        # mat1 and mat2 shapes cannot be multiplied (1x32 and 26x200)
        # conclusion: initial value 6 comes from the initial is to match the mat1
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=[self._env.observation_space.shape[0] + 16], dtype=np.float32)#env.observation_space
        self.action_space = self._env.action_space
        # CRAS14 to see the dimensions of self.action_space and self.observation_space
        # print("action space: ",self.action_space)
        # print("obervation space: ", self.observation_space)
        self._initial_state = self._env.reset()

        if self._record_video:
            self._video_recorder = BestEpisodesVideoRecorder(path=config['data_folder_experiment'], max_videos=5)

        # Which dimensions in the state vector are design parameters?
        self._design_dims = list(range(self.observation_space.shape[0] - len(self._current_design), self.observation_space.shape[0]))
        assert len(self._design_dims) == 4

    def render(self):
        pass

    # The objective is to create a way for the program to communicate with the REAL environment

    # CRAS14
    # sends an action to the robot as NDArray (normalized values preferred for RL)
    # return the current state of the robot and the reward for the action taken in a form of NDArray
    # the reward is the distance travelled by the robot
    def step(self, a):
        info = {}
        state, reward, done, _ = self._env.step(a)
        state = np.append(state, self._config_numpy)
        # CRAS14 The are confiq_numpy are the only optimized parameters
        info['orig_action_cost'] = 0.1 * np.mean(np.square(a))
        info['orig_reward'] = reward

        if self._record_video:
            self._video_recorder.step(env=self._env, state=state, reward=reward, done=done)

        # CRAS14 to see the dimensions of state and a
        # print("state: ", state)
        # print("action: ", a)

        return state, reward, False, info

    # CRAS14
    # resets the robot to initial position
    # basically does the same as step() but without the action?
    def reset(self):
        state = self._env.reset()
        self._initial_state = state
        state = np.append(state, self._config_numpy)

        if self._record_video:
            self._video_recorder.reset(env=self._env, state=state, reward=0, done=False)

        return state

    def set_new_design(self, vec):
        self._env.reset_design(vec)
        self._current_design = vec
        self._config_numpy = np.array(vec)

        if self._record_video:
            self._video_recorder.increase_folder_counter()
    
    # CRAS14: this function pauses the program before every new design
    def pausing(self, coadapt):
        value = None
        while value != "y" and value != "n": 
            value = input("Continue? (y/n): ")
        
        # CRAS14: if the user types "y" the program will continue
        if value == "y":
            return 1
        elif value == "n":
            # coadapt.save_logged_data()
            # coadapt.save_networks()
            if coadapt._design_counter <= 1:
                print("No designs to optimize")
                return 0

            # CRAS14: Ask for saving evoreplay data
            save_ans = input("Save current evoreplay values? (y/n): ")
            if save_ans == "n":
                return 0
            print("COADAPT REPLAY")
            # CRAS14: if the user types "n" the program will stop
            # save the data from evoreplay
            shorter_list, longer_list = coadapt._replay.get_init_params()
            
            # paths
            path_short = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../evoreplay_saves/short_save")
            path_species = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../evoreplay_saves/species")
            path_population = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../evoreplay_saves/population")
            path_init_state = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../evoreplay_saves/init_state")

            # CRAS14: saving the data in chunks
            self.save_evoplay_data(shorter_list, path_short)
            self.save_evoplay_data(longer_list[0], path_species)
            self.save_evoplay_data(longer_list[1], path_population)
            self.save_evoplay_data(longer_list[2], path_init_state)

            return 0

    # CRAS14: method that saves previous evoreplay data
    def save_evoplay_data(self,list, path):
        
        print("Saving evoreplay data, to", path)
        # path to a directory using os
        for i in range(len(list)):
            chunk = list[i]
            file_name = os.path.join(path, f"{i}_chunk.npy")
            np.save(file_name, chunk)
            print(f"Chunck {i} in list saved.")
        print("Evoreplay data saving complete")
        return


    # CRAS14: method that load previous evoplay data
    def load_evoplay_data(self, coadapt):
        """ Loads previous evoplay data.

        The function loads previous evoplay data from a file.

        Args:
            path: String stating the path to the file.

        """
        path_short = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../evoreplay_saves/short_save")
        path_species = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../evoreplay_saves/species")
        path_population = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../evoreplay_saves/population")
        path_init_state = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../evoreplay_saves/init_state")

        lista_long = []
        lista_short = self.load_data(path_short)
        lista_long.append(self.load_data(path_species))
        lista_long.append(self.load_data(path_population))
        lista_long.append(self.load_data(path_init_state))
        coadapt._replay.set_init_params(lista_short, lista_long)
        return

    # CRAS14: loading data method
    def load_data(self, path):
        lista = []
        print("Loading evoreplay data, from", path)
        for filename in sorted(os.listdir(path)):
            if filename.endswith(".npy"):
                file_path = os.path.join(path, filename)
                chunk = np.load(file_path, allow_pickle=True)
                lista.append(chunk)

        print("Evoreplay data loading complete")
        return lista


    ## def save_current_design()
    ## reset the robot position, and legs orientation
    # CRAS14 Memos: give new parameters to the robot, NOTE the size parameter depends on amount of legs
    def get_random_design(self):
        optimized_params = np.random.uniform(low=0.8, high=2.0, size=4)
        return optimized_params

    def get_current_design(self):
        return copy.copy(self._current_design)

    def get_design_dimensions(self):
        return copy.copy(self._design_dims)

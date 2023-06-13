import gym
from robot import Robot
import numpy as np
import copy
import time
import os


class RobotEnv(gym.Env):

    def __init__(self):
        
        self.robot = Robot()
        self.robot.open_all_connections() # Attaches servos also.
        
        self._current_design = [1] * 8
        self._config_numpy = np.array(self._current_design)
        self.design_params_bounds = [(0.5, 2.0)] * 8  # CRAS14 limiter for designs, both tip radius and length scaled between these


        number_of_design_parameters = len(self._current_design)

        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(8,))
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=[24 + 14 + number_of_design_parameters])

        # Init params: 
        # 1. White legs from box with length 6.5 cm from attachment
        # 2. First and second legs replaced by black legs from box with length 7.5 cm from attachment
        # 3. All legs replaced by black legs from box
        
        self.init_sim_params = [[1]*8,
                                [7.5/6.5, 1, 7.5/6.5, 1, 1, 1, 1, 1],
                                [7.5/6.5, 1]*4]

        self._intial_state = self.reset()

        self._design_dims = list(range(self.observation_space.shape[0] - len(self._current_design), self.observation_space.shape[0]))
        assert len(self._design_dims) == 8

        print("Env init complete")
        

    def step(self, action):

        self.robot.apply_action(action)
        time.sleep(0.1)
        state, reward = self.robot.calc_state()
        state = np.append(state, self._config_numpy)
        info = {}
        info['orig_action_cost'] = 0.1 * np.mean(np.square(action)) # FROM HALFCHEETAHENV
        info['orig_reward'] = reward

        #self.robot.print_state()

        return state, reward, False, info
    
    def reset(self):
        
        self.robot.reset() # There is already one second sleep in robot.reset()
        state, _ = self.robot.calc_state()
        state = np.append(state, self._config_numpy)

        return state
  
    def close(self):
        self.robot.close_all_connections() # Disables servos also

    def pausing(self, coadapt, next_params):
        
        _ = self.reset() # ROBOTENV, to ease the changing of leg segments 


        value = None
        while value != "y" and value != "n": 
            value = input("Continue with design: {}? (y/n): ".format(next_params))
        
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


    


    def get_random_design(self): # Used in design pso?
        optimized_params = np.random.uniform(low=self.design_params_bounds[0][0], high=self.design_params_bounds[0][1], size=8)
        return optimized_params
   
    def get_current_design(self):
        return copy.copy(self._current_design)

    def get_design_dimensions(self):
        return copy.copy(self._design_dims)

    def set_new_design(self, vec): 

        # set the new design irl. 

        self._current_design = vec
        self._config_numpy = np.array(vec)
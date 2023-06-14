import sys
sys.path.append("/home/janix/Coadaption_with_RealAnt/project14-COADAPT/Coadaptation/realant_environment") #copy path full path of realant_environment
from realant_environment.robot_env import RobotEnv
from RL.soft_actor import SoftActorCritic
from DO.pso_batch import PSO_batch
from DO.pso_sim import PSO_simulation
from Environments import evoenvs as evoenvs
import utils
import time
from RL.evoreplay import EvoReplayLocalGlobalStart
import numpy as np
import os
import csv
import torch

def select_design_opt_alg(alg_name):
    """ Selects the design optimization method.

    Args:
        alg_name: String which states the design optimization method. Can be
            `pso_batch` or `pso_sim`.

    Returns:
        The class of a design optimization method.

    Raises:
        ValueError: If the string alg_name is unknown.
    """
    if alg_name == "pso_batch":
        return PSO_batch
    elif alg_name == "pso_sim":
        return PSO_simulation
    else:
        raise ValueError("Design Optimization method not found.")

# CRAS14 memos: houd return class of theproject environment
def select_environment(env_name):
    """ Selects an environment.

    Args:
        env_name: Name (string) of the environment on which the experiment is to
            be executed. Can be `HalfCheetah`.

    Returns:
        The class of an environment

    Raises:
        ValueError: If the string env_name is unknown.

    """
    if env_name == 'HalfCheetah':
        return evoenvs.HalfCheetahEnv
    elif env_name == 'RealAnt':
        return RobotEnv
    else:
        raise ValueError("Environment class not found.")

def select_rl_alg(rl_name):
    """ Selectes the reinforcement learning method.

    Acoadaptrgs:
        rl_name: Name (string) of the rl method.

    Returns:
        The class of a reinforcement learning method.

    Raises:
        ValueError: If the string rl_name is unknown.
    """
    if rl_name == 'SoftActorCritic':
        return SoftActorCritic
    else:
        raise ValueError('RL method not fund.')

class Coadaptation(object):
    """ Basic Co-Adaptaton class.

    """

    def __init__(self, config):
        """
        Args:
            config: A config dictonary.

        """

        self._config = config
        utils.move_to_cuda(self._config)

        # TODO This should not depend on rl_algorithm_config in the future
        self._episode_length = self._config['steps_per_episodes']
        self._reward_scale = 1.0 #self._config['rl_algorithm_config']['algo_params']['reward_scale']

        # CRAS14 memos: replace "evoenvs" environemnt
        self._env_class = select_environment(self._config['env']['env_name'])
        self._env = RobotEnv() #evoenvs.HalfCheetahEnv(config=self._config)

        # CRAS14 memos:
        # save these values for later use
        self._replay = EvoReplayLocalGlobalStart(self._env,
            max_replay_buffer_size_species=int(1e6),
            max_replay_buffer_size_population=int(1e7))

        self._rl_alg_class = select_rl_alg(self._config['rl_method'])

        self._networks = self._rl_alg_class.create_networks(env=self._env, config=config)

        self._rl_alg = self._rl_alg_class(config=self._config, env=self._env , replay=self._replay, networks=self._networks)

        self._do_alg_class = select_design_opt_alg(self._config['design_optim_method'])
        # CRAS14 memos: config and replay used
        self._do_alg = self._do_alg_class(config=self._config, replay=self._replay, env=self._env)

        # if self._config['use_cpu_for_rollout']:
        #     utils.move_to_cpu()
        # else:
        #     utils.move_to_cuda(self._config)
        # # TODO this is a temp fix - should be cleaned up, not so hppy with it atm
        # self._policy_cpu = self._rl_alg_class.get_policy_network(SoftActorCritic.create_networks(env=self._env, config=config)['individual'])
        utils.move_to_cuda(self._config)

        self._last_single_iteration_time = 0
        self._design_counter = 0
        self._episode_counter = 0
        self._data_design_type = 'Initial'

    def initialize_episode(self):
        """ Initializations required before the first episode.

        Should be called before the first episode of a new design is
        executed. Resets variables such as _data_rewards for logging purposes
        etc.

        """
        # self._rl_alg.initialize_episode(init_networks = True, copy_from_gobal = True)
        self._rl_alg.episode_init()
        self._replay.reset_species_buffer()



        self._data_rewards = []
        self._episode_counter = 0


    def single_iteration(self):
        """ A single iteration.

        Makes all necessary function calls for a single iterations such as:
            - Collecting training data
            - Executing a training step
            - Evaluate the current policy
            - Log data

        """
        print("Time for one iteration: {}".format(time.time() - self._last_single_iteration_time))
        self._last_single_iteration_time = time.time()
        self._replay.set_mode("species")
        #self.collect_training_experience()
        Coadaptation.collect_training_experience(self)
        # TODO Change here to train global only after five designs; CRAS14 set it to 3
        train_pop = self._design_counter > 3
        if self._episode_counter >= self._config['initial_episodes']:
            self._rl_alg.single_train_step(train_ind=True, train_pop=train_pop)
        self._episode_counter += 1
        self.execute_policy()
        self.save_logged_data()
        self.save_networks()

    def collect_training_experience(self):
        print("training...")
        """ Collect training data.

        This function executes a single episode in the environment using the
        exploration strategy/mechanism and the policy.
        The data, i.e. state-action-reward-nextState, is stored in the replay
        buffer.

        """
        state = self._env.reset()
        nmbr_of_steps = 0
        done = False

        if self._episode_counter < self._config['initial_episodes']:
            policy_gpu_ind = self._rl_alg_class.get_policy_network(self._networks['population'])
        else:
            policy_gpu_ind = self._rl_alg_class.get_policy_network(self._networks['individual'])
        # self._policy_cpu = utils.copy_network(network_to=self._policy_cpu, network_from=policy_gpu_ind, config=self._config, force_cpu=self._config['use_cpu_for_rollout'])
        self._policy_cpu = policy_gpu_ind

        if self._config['use_cpu_for_rollout']:
            utils.move_to_cpu()
        else:
            utils.move_to_cuda(self._config)

        while not(done) and nmbr_of_steps <= self._episode_length:
            nmbr_of_steps += 1
            action, _ = self._policy_cpu.get_action(state)
            new_state, reward, done, info = self._env.step(action)
            # TODO this has to be fixed _variant_spec
            reward = reward * self._reward_scale
            terminal = np.array([done])
            reward = np.array([reward])
            self._replay.add_sample(observation=state, action=action, reward=reward, next_observation=new_state,
                           terminal=terminal)
            state = new_state
        self._replay.terminate_episode()
        utils.move_to_cuda(self._config)
        print("training ended")

    def execute_policy(self):
        """ Evaluates the current deterministic policy.

        Evaluates the current policy in the environment by unrolling a single
        episode in the environment.
        The achieved cumulative reward is logged.

        """
        print("executing policy...")
        state = self._env.reset()
        done = False
        reward_ep = 0.0
        reward_original = 0.0
        action_cost = 0.0
        nmbr_of_steps = 0

        if self._episode_counter < self._config['initial_episodes']:
            policy_gpu_ind = self._rl_alg_class.get_policy_network(self._networks['population'])
        else:
            policy_gpu_ind = self._rl_alg_class.get_policy_network(self._networks['individual'])
        # self._policy_cpu = utils.copy_network(network_to=self._policy_cpu, network_from=policy_gpu_ind, config=self._config, force_cpu=self._config['use_cpu_for_rollout'])
        self._policy_cpu = policy_gpu_ind

        if self._config['use_cpu_for_rollout']:
            utils.move_to_cpu()
        else:
            utils.move_to_cuda(self._config)

        while not(done) and nmbr_of_steps <= self._episode_length:
            nmbr_of_steps += 1
            action, _ = self._policy_cpu.get_action(state, deterministic=True)
            new_state, reward, done, info = self._env.step(action)
            action_cost += info['orig_action_cost']
            reward_ep += float(reward)
            reward_original += float(info['orig_reward'])
            state = new_state
        utils.move_to_cuda(self._config)
        # Do something here to log the results
        self._data_rewards.append(reward_ep)
        print("policy execution ended")


    def save_networks(self):
        """ Saves the networks on the disk.
        """
        if not self._config['save_networks']:
            return

        checkpoints_pop = {}
        for key, net in self._networks['population'].items():
            checkpoints_pop[key] = net.state_dict()

        checkpoints_ind = {}
        for key, net in self._networks['individual'].items():
            checkpoints_ind[key] = net.state_dict()

        checkpoint = {
            'population' : checkpoints_pop,
            'individual' : checkpoints_ind,
        }
        file_path = os.path.join(self._config['data_folder_experiment'], 'checkpoints')
        if not os.path.exists(file_path):
          os.makedirs(file_path)
        torch.save(checkpoint, os.path.join(file_path, 'checkpoint_design_{}.chk'.format(self._design_counter)))

    # test this function by printing the network parameters before and after loading
    # and compare the before and after
    def load_networks(self, path):
        """ Loads networks from the disk.
        """
        model_data = torch.load(path) #, map_location=ptu.device)

        model_data_pop = model_data['population']
        for key, net in self._networks['population'].items():
            params = model_data_pop[key]
            net.load_state_dict(params)
        

        model_data_ind = model_data['individual']
        for key, net in self._networks['individual'].items():
            params = model_data_ind[key]
            net.load_state_dict(params)
        
        print("load network complete")

    def save_logged_data(self):
        """ Saves the logged data to the disk as csv files.

        This function creates a log-file in csv format on the disk. For each
        design an individual log-file is creates in the experient-directory.
        The first row states if the design was one of the initial designs
        (as given by the environment), a random design or an optimized design.
        The second row gives the design parameters (eta). The third row
        contains all subsequent cumulative rewards achieved by the policy
        throughout the reinforcement learning process on the current design.
        """
        file_path = self._config['data_folder_experiment']
        current_design = self._env.get_current_design()

        with open(
            os.path.join(file_path,
                'data_design_{}.csv'.format(self._design_counter)
                ), 'w') as fd:
            cwriter = csv.writer(fd)
            cwriter.writerow(['Design Type:', self._data_design_type])
            cwriter.writerow(current_design)        # CRAS14 memos: write the design of the model
            cwriter.writerow(self._data_rewards)    # CRAS14 memos: write the the data_rewards to the file

    def run(self):
        """ Runs the Fast Evolution through Actor-Critic RL algorithm.

        First the initial design loop is executed in which the rl-algorithm
        is exeuted on the initial designs. Then the design-optimization
        process starts.
        It is possible to have different numbers of iterations for initial
        designs and the design optimization process.
        """
        iterations_init = self._config['iterations_init']
        iterations = self._config['iterations']
        design_cycles = self._config['design_cycles']
        exploration_strategy = self._config['exploration_strategy']

        # CRAS14 TODO: maybe catch KeyboardInterrupt and close robot connections before exiting 

        print("Running initial design loop")
        pause_value = self._intial_design_loop(iterations_init)

        # CRAS14 Continue to optimization
        if pause_value == 0 or pause_value == None:
            continue_to_optimization = None
            while continue_to_optimization != "y" and continue_to_optimization != "n":
                continue_to_optimization = input("Continue to robot design optimization? (y/n)?: ")
            if continue_to_optimization == "y":
                self._training_loop(iterations, design_cycles, exploration_strategy)
                return
        # Program ends 
        
        # CRAS14 INTEGRATION removed the rest from the else condition and added env.close()

        self._env.close()
        print("Program ended")
        return

    def _training_loop(self, iterations, design_cycles, exploration_strategy):
        """ The trianing process which optimizes designs and policies.

        The function executes the reinforcement learning loop and the design
        optimization process.

        Args:
            iterations: An integer stating the number of iterations/episodes
                to be used per design during the reinforcement learning loop.
            design_cycles: Integer stating how many designs to evaluate.
            exploration_strategy: String which describes which
                design exploration strategy to use. Is not used at the moment,
                i.e. only the (uniform) random exploration strategy is used.

        """
        self.initialize_episode()
        # TODO fix the following
        initial_state = self._env.reset() #CRAS 14, unmodified: self._env._env.reset()

        self._data_design_type = 'Optimized'

        optimized_params = self._env.get_random_design()
        q_network = self._rl_alg_class.get_q_network(self._networks['population'])
        policy_network = self._rl_alg_class.get_policy_network(self._networks['population'])
        # CRAS14 memos: this might need to be saved
        optimized_params = self._do_alg.optimize_design(design=optimized_params, q_network=q_network, policy_network=policy_network)
        optimized_params = list(optimized_params)

        for i in range(design_cycles):
            self._design_counter += 1

            print("Design number: ", self._design_counter)
            # CRAS14: Optimization pausing
            print("In optimization loop, walking iteration")
            continue_value = self._env.pausing(self, optimized_params)
            if continue_value == 1:
                self._env.set_new_design(optimized_params)
            else:
                print("Stopped at the optimization training loop")
                self._env.close()
                return
            

            # CRAS14 memos: Walking iteration
            # Reinforcement Learning
            for _ in range(iterations):
                self.single_iteration()
            
            # CRAS14: Implement a method to give the option to skip the optimization process
            optimization_ans = None
            while optimization_ans != "n" and optimization_ans != "y":
                optimization_ans = input("Continue to new robot design optimization? (y/n): ")
            if optimization_ans == "y":
                print("In design optimization")
            # Design Optimization
                if i % 2 == 1:
                    self._data_design_type = 'Optimized'
                    q_network = self._rl_alg_class.get_q_network(self._networks['population'])
                    policy_network = self._rl_alg_class.get_policy_network(self._networks['population'])
                    optimized_params = self._do_alg.optimize_design(design=optimized_params, q_network=q_network, policy_network=policy_network)
                    optimized_params = list(optimized_params)
                else:
                    self._data_design_type = 'Random'
                    optimized_params = self._env.get_random_design()
                    optimized_params = list(optimized_params)
            self.initialize_episode()

    def _intial_design_loop(self, iterations):
        """ The initial training loop for initial designs.

        The initial training loop in which no designs are optimized but only
        initial designs, provided by the environment, are used.

        Args:
            iterations: Integer stating how many training iterations/episodes
                to use per design.

        """
        # CRAS14: Ask whether to use previous network
        prev_network = None
        while prev_network != "y" and prev_network != "n":
            prev_network = input("Use previous Network? (y/n): ")
        if prev_network == "y":
            prev_network = input("Enter previous network path: ")
            try:
                self.load_networks(prev_network)
            except:
                print("Error: Could not load network")
                return -1

        # CRAS14: Ask whether to use previous evoreplay data
        prev_evoplay = None
        while prev_evoplay != "y" and prev_evoplay != "n":
            prev_evoplay = input("Use previous evoplay data? (y/n): ")
        if prev_evoplay == "y":
           self._env.load_evoplay_data(self)

        self._data_design_type = 'Initial'
        for params in self._env.init_sim_params:
            self._design_counter += 1

            print("Design number: ", self._design_counter)
            # CRAS14: Pausing the program
            print("Initial design optimization: ")
            pause_value = self._env.pausing(self, params)
            if pause_value == 0:
                return pause_value
            self._env.set_new_design(params)
            self.initialize_episode()

            # Reinforcement Learning
            for _ in range(iterations):
                self.single_iteration()



    



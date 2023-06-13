# Introduction

This documentation presents the modifications made to a program based on the co-adaptation program described in a paper presented at the [Conference on Robot Learning in 2019](https://sites.google.com/view/drl-coadaptation/home). The document explores the changes implemented and discusses how the program operates with the new modifications. It also includes the installation process, and highlights encountered errors during the project work. We want to acknowledge one of the original co-adaptation program's authors. 

## Acknowledgements
We thank the instructor, Kevin Sebastian Luck, for his guidance and permission to utilize their team's work.
The co-adaptation program extensively uses rlkit, a framework developed and maintained by Vitchyr Pong, and the original can be found [here](https://github.com/rail-berkeley/rlkit). We have also made small modifications to rlkit due to Numpy compatibility, which is also in this repository.

## Co-Adaptation of Morphology and Behaviour
One of the prominent features of the quadruped robot is the integration of Co-Adaptation of Morphology and Behaviour with Deep Reinforcement Learning. This approach combines the Soft-Actor-Critic (SAC) algorithm with the Optimization Algorithm Particle Swarm Optimization (PSO) to enable the co-adaptation of the robot's morphology and behavior through deep reinforcement learning.

## Pausing Method
We have implemented a pausing method in the program to enhance flexibility and control during adaptation. After each iteration, the program can be paused, allowing the option to terminate the program and save the current state or skip a part of the adaptation process. This feature proves particularly useful when there is a need to train the robot solely for walking, enabling the skipping of certain processes such as design optimization.

## Saving Method
A saving method has been incorporated into the program to facilitate continuity and accommodate changes to the robot's parts. This method enables the program to save its current state, ensuring that progress is not lost when modifications are made. By saving the program's state, users can resume where they left off, even after altering the robot's components. During program initialization, users are prompted to decide whether to load previous data, including reinforcement learning data and other information saved from the last program run. This flexibility allows for fresh runs of the program when required.

## Modifications and Memo Labels
Throughout this project, various modifications have been made to the program, each labeled `CRAS14`. These modifications range from improvements to core functionality to adjustments to address specific requirements. It is important to note that some modifications are marked as `memos` with `CRAS14`, indicating that they serve as internal notes for reference and clarification purposes.

## Installation
Initial installation steps are as follows:

1. Clone this repository to your local device. Install the required libraries and packades via pip via `pip3 install -r requirements.txt.`

* Note that compared with the original co-adaptation repository, the requirement list has been modified due to compatibility issues with the newer version of certain packages.

2. Install Anaconda, any version should be fine.

3. Install Mujoco, through [this](https://www.youtube.com/watch?v=Wnb_fiStFb8) instruction, which has a written step-by-step guide in the description if you want to run co-adaptation in a simulation. Install mujoco-py regularly.

4. Within conda environment install PyTorch.

* If you don't have or don't want to use GPU available, install the CPU version of the package.

5. Install GYM v0.21.0; DO NOT install GYM using PIP INSTALL; The new interface is not compatible with. Instead, install GYM using the following command: `pip3 install gym==0.21.0`


## Setting up the environment
Set `PYTHONPATH` with 


    export PYTHONPATH=/path/to/rlkit/

where the folder `/path/to/rlkit` contains the folder `rlkit`. This enables us to import rlkit with `import rlkit`. You may need to set the `PYTHONPATH` every time after launching a new terminal.

## Running the program
After the setup, run either two available configurations in `experiment_configs.py` by running either of the commands:

    python3 main.py sac_pso_batch

or 

    python3 main.py sac_pso_sim # Simulation version

You can change co-adaptation parameters from the `experiment_configs.py`. 

The program saves and load all evoreplay data to a directory `named evoreplay_saves` that has 
* `init_state`
* `population`
* `short_save`
* `species directories`

as subdirectories.

Ensure the `evoreplay_saves` directory and its subdirectories exist within the `Coadaptation` directory for the saving and loading function to work.

When the program asks for a previous network path, enter the network path you want to load.

## Running the program
The original implementation of the co-adaptation program will automatically create directories that store performance and achieve rewards. For more information about this, refer to the original program repository in the section [Data logging](https://github.com/eicio/CRAS-14-Final/tree/coadapt-save-load-all/project14-COADAPT/Coadaptation#data-logging)

## Known errors and solutions
If you encounter the following error when trying to create a new environment in Anaconda:

    LD_PRELOAD cannot be preloaded (cannot open shared object file): ignored

A possible solution to resolve the error:

    apt-get install glib2.0
    sudo apt-get install glib2.0
You can also try:

    pip install -e . --no-cache

which didn't affect the outcome. You can also try to follow the instructions [here](https://askubuntu.com/questions/1054508/how-to-set-so-to-be-available-for-ld-preload) to resolve the error.



If encountering the following error:

    ImportError: ...anaconda3/envs/mujoco_py/bin/../lib/libstdc++.so.6: version `GLIBCXX_3.4.30' not found (required by /lib/x86_64-linux-gnu/libLLVM-15.so.1)

Try the following:

    conda install -c conda-forge gcc=12.1.0

It may take a minute to complete. The original [solution](https://stackoverflow.com/questions/72540359/glibcxx-3-4-30-not-found-for-librosa-in-conda-virtual-environment-after-tryin).

If receive the following error:

    E: Couldn't find any package by glob 'libglew1.13'

    E: Couldn't find any package by regex 'libglew1.13'
Solution that worked was found [here](https://otland.net/threads/debian-installing-library-to-compile-otclient-failed.253692/).


We discovered that the newest NumPy (numpy-1.24.3) causes the following error:
    
    File ".../project14-COADAPT/rlkit/rlkit/torch/core.py", line 49, in _filter_batch
        if v.dtype == np.bool:
    File ".../anaconda3/envs/mujoco_py/lib/python3.8/site-packages/numpy/__init__.py", line 305, in __getattr__
        raise AttributeError(__former_attrs__[attr])
    AttributeError: module 'numpy' has no attribute 'bool'.
    `np.bool` was a deprecated alias for the builtin `bool`. To avoid this error in existing code, use `bool` by itself. Doing this will not modify any behavior and is safe. If you specifically wanted the numpy scalar type, use `np.bool_` here.

There are two possible solutions to this,
1. Downgrade NumPy, a solution that worked can be found [here](https://stackoverflow.com/questions/74893742/how-to-solve-attributeerror-module-numpy-has-no-attribute-bool).

2. Change the all `np.bool` to `bool` in the file `core.py` in the folder `rlkit/rlkit/torch/`.

## WSL2
During the project, WSL2 was used to work on the program files due to being unable to access the native Ubundu operation environment—both regular installations of Ubuntu 20.04 LTS and Ubuntu 22.04.2 LTS versions on WSL2. If you want to run a visual simulation with Ubuntu 22.04.2 LTS version, follow this [guide](https://www.youtube.com/watch?v=7Sym3uL6YWo&t). Visualization on Ubuntu 20.04 LTS can be done by following this [guide](https://github.com/davidbombal/wsl2/blob/main/ubuntu_gui_youtube).

## Other features
Functions not labeled with `CRAS14` in the program are from the original version of the program; refer to the original [repository](https://github.com/ksluck/Coadaptation).

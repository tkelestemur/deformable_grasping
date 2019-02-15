# ur_mujoco

### Installation
* Install [MuJoCo 200](https://www.roboti.us/index.html) and set the environment variables as follows:
    - `export MUJOCO200_HOME=/path/to/mujoco200_dist`
    - `export MUJOCO_KEY=/path/to/mujoco_key.txt`
* `git clone https://github.com/tkelestemur/ur_mujoco.git`
* `cd ur_mujoco && mkdir build && cd build`
* `cmake .. && make`
* `./basic`

#### TODO
- [ ] Write a grasping procedure for data collection
- [ ] Write object-oriented code
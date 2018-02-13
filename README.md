# Open Ai Gym extension for robotics
Inspired by [gym_gazebo](https://github.com/erlerobot/gym-gazebo), robotics_gym is a customized gym extension that includes environments for gazebo and stage simulator.
## Installation
### Requirements
+ ROS Kinetics
+ Player (3.02) and Stage (4.1.1) with python support. 
+ OpenAI Gym

## Player/Stage installation
[How to use Player/Stage](http://player-stage-manual.readthedocs.io/en/latest/)
### Player
```bash
$ git clone https://github.com/playerproject/player
$ cd player
$ mkdir build
$ cd build
$ cmake -jn -DBUILD_PYTHONCPP_BINDINGS=ON ..
$ sudo make install -jn
```
### Stage
```bash
$ git clone https://github.com/rtv/Stage
$ cd Stage
$ mkdir build
$ cd build
$ cmake -jn ..
$ sudo make install -jn
```
#### Execution
```bash
$ export LD_LIBRARY_PATH=/usr/local/lib64/:$LD_LIBRARY_PATH
$ cd /usr/local/share/stage/worlds
$ player simple.cfg
```
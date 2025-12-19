# DreamWaQ

## Description
This repo contains implementation of the paper [Learning Robust Quadrupedal Locomotion With Implicit Terrain Imagination via Deep Reinforcement Learning](https://arxiv.org/abs/2301.10602)

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
  
## Installation
This repo requires the following packages:
- [Isaac Gym](https://developer.nvidia.com/isaac-gym)
- Legged Gym
- RSL-RL

To install Isaac Gym, go to the link and follow the instructions on the page.

1. clone this repo

2. 
```bash
  cd rsl_rl-1.0.2
  pip install -e .
  cd ..
```
3. 
```bash
  cd legged_gym
  pip install -e .
  cd ..
```

## Usage
To train your robot run
```bash
    python3 legged_gym/scripts/train.py --task=[robot name]  
```  

To evaluate the trained policy run
```bash
    python3 legged_gym/scripts/play.py --task=[robot name]
```  

## Configuration
Requires python 3.8 and numpy version<=1.24.


## Sincerely thank the original authors of the repositories:

1. https://github.com/Manaro-Alpha/DreamWaQ
2. https://github.com/ShengqianChen/DreamWaQ_Go2W 
3. https://github.com/LucienJi/MetaRobotics
4. https://github.com/InternRobotics/HIMLoco
5. https://github.com/yusongmin1/My_unitree_go2_gym

我训练go2时是先在平地训练1500代左右，然后拆掉类似于default pos这样的奖励，移动到trimesh训练
我训了3500+3500代左右就差不多能上15cm的楼梯了，但是再接着训练，奖励值就会往下掉，不知道为什么。

如果只是换一个四足（12自由度）训练的话问题不大。

如果要改模型（尤其是电机数量不是12的），并且会涉及到本体观测值、特权观测值的维度改变的话，需要修改的地方会很多
rsl_rl中，ppo.py，会从特权观测值里切出base_lin_vel，用于VAE的训练，切片的维度要改，helper.py里的
PolicyExporterDWAQ这个东西也是要改的，然后是num_obs_hist，这个值也不建议改，我记得on_policy_runner.py里好像也写死了这个玩意。

这个没有在实物上测试过。

可能还是存在一些小问题。

目前的privileged_obs可能不太好，也许要加入随机化参数之类的？


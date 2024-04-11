# Manipulation Package

## 1. Installation

```buildoutcfg
chmod +x ./install.sh && ./install.sh
```
## 2. Description

## 3. Usage

## 4. Roadmap

## Movement actions

Send actions by publishing a message ont the `/Movement/Goal` topic :

```bash
rostopic pub /Movement/goal manipulation_pepper/MovementActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  order: '<ACTION_NAME>'
  target: 0"
```

the target parameter is currently unused but is used for testing and future behaviors

Replace <ACTION_NAME> with one of the possible actions

list of possible actions

```
set_hand
pose_middle
pose_dialog
pose_pregrasp
pose_restaurant
raise_hand
grab_2arms
release_grab_2arms
grab_bag
put_down_bag
```

## Citations

If you use this project, please consider citing:

```
@incollection{buche2023robocup,
  title={RoboCup@ Home SSPL Champion 2023: RoboBreizh, a Fully Embedded Approach},
  author={Buche, C{\'e}dric and Neau, Ma{\"e}lic and Ung, Thomas and Li, Louis and Wang, Sinuo and Bono, C{\'e}dric Le},
  booktitle={Robot World Cup},
  pages={374--385},
  year={2023},
  publisher={Springer}
}

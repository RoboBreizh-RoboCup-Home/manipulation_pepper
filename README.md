# Manipulation Package

## 1. Installation

```buildoutcfg
chmod +x ./install.sh && ./install.sh
```
## 2. Description

## 3. Usage

## 4. Roadmap

## Movement actions

Possible actions : pose_middle, pose_dialog, raise_hand, pose_restaurant, stop_pose_restaurant

Send actions with rosrun :

```bash
rosrun manipulation_pepper movement_client.py <ACTION_NAME>
```

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
  order: '<ACTION_NAME>'"
```

Replace <ACTION_NAME> with one of the possible actions (pose_middle, pose_dialog, raise_hand, pose_restaurant, stop_pose_restaurant)

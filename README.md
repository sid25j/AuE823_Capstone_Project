# Autonomous Bot
![IMG_6640](https://user-images.githubusercontent.com/57304529/166848576-27cb1e29-65ac-468f-a5a9-19adac682aff.jpg)

# Project Objectives
- Four different tasks to be done in simulation and simulation look-a-like real world 
- Task 1: Wall following & Obstacle Avoidance
- Task 2: Line following
- Task 3: Stop sign detection
- Task 4: April Tag Following
- Overall integration

# Simulation

```
$ roscore
$ roslaunch aue_finals turtlebot3_autonomy_final.launch
$ roslaunch aue_finals apriltag_detection.launch
$ roslaunch aue_finals april_tag_test.launch
```


https://user-images.githubusercontent.com/57304529/166850430-0c463c3a-b923-4917-afc9-2a4afa85c045.mp4


# Sim2Real

```
$ roslaunch aue_finals turtlebot3_autonomy_final_real.launch
$ rosrun image_transport republish compressed in:=camera/image raw out:=camera/image_raw
```


https://user-images.githubusercontent.com/57304529/166852070-9cbb2104-f5cc-448e-8a5c-914f97d14f4b.mp4



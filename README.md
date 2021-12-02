# roomba
Final project of Group 6, Intelligent Robotics 2021, UoB


## TODOs
- /roomba/charging_pos
- /roomba/map_ready

## [Proposal](https://bham-my.sharepoint.com/:w:/r/personal/mxl367_student_bham_ac_uk/_layouts/15/Doc.aspx?sourcedoc=%7BCBCD51A3-804D-42BA-A23D-9412685EAAF5%7D&file=Intel.%20Robotics%20Project%20Proposal%20-%20Group%206.docx&action=edit&mobileredirect=true&wdPreviousSession=54702ad2-d520-4f52-a1b5-e6ba7baac8fe&wdOrigin=TEAMS-ELECTRON.p2p.undefined)

## useful commands

```bash
#setups for dependency
cd <catkin_ws>/src
git clone https://github.com/UOBIR06/ipa_coverage_planning.git  # on dev branch
# or for ssh:  git clone git@github.com:UOBIR06/ipa_coverage_planning.git
rosdep update
rosdep install --from-paths src --ignore-src
#rosdep install roomba
cd <catkin_ws>
catkin_make # need to do this each time you pull from ipa_coverage_planning cuz it's wirtten in c++
catkin_make --pkg ipa_room_segmentation
```

```bash
# demo of room_exploration given png map
roslaunch roomba room_exploration.launch
```
```bash
# demo of frontier exploration
roslaunch roomba nav.launch
rosrun roomba explore.py
```
## practice
when publish/subscribe topics only used for this program. use `/roomba/` namespace to avoid conflicts.

[comment]: <> (#### In simulated world:)

[comment]: <> (The localisation node can be tested in stage simulation &#40;without the need for robot&#41;.)

[comment]: <> (        roscore)

[comment]: <> (        rosrun map_server map_server <catkin_ws>/map.yaml)

[comment]: <> (        rosrun stage_ros stageros <catkin_ws>/src/socspioneer/data/meeting.world)

[comment]: <> (        roslaunch socspioneer keyboard_teleop.launch  # ---- run only if you want to move robot using keyboard)

[comment]: <> (        rosrun pf_localisation node.py    # ----- requires laser_trace, and completed pf.py methods.)

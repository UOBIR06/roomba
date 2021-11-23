# roomba
Final project of Group 6, Intelligent Robotics 2021, UoB

## [Proposal](https://bham-my.sharepoint.com/:w:/r/personal/mxl367_student_bham_ac_uk/_layouts/15/Doc.aspx?sourcedoc=%7BCBCD51A3-804D-42BA-A23D-9412685EAAF5%7D&file=Intel.%20Robotics%20Project%20Proposal%20-%20Group%206.docx&action=edit&mobileredirect=true&wdPreviousSession=54702ad2-d520-4f52-a1b5-e6ba7baac8fe&wdOrigin=TEAMS-ELECTRON.p2p.undefined)

## useful commands

```bash
#setups for dependency
rosdep update
rosdep install roomba
catkin_make
```

```bash
# start the project:
roslaunch pf_localisation pf_mcl.launch
# use the bag1 files
roslaunch pf_localisation bag.launch bag:=1
# use the bag2 files
roslaunch pf_localisation bag.launch bag:=2
```

#### In simulated world:

The localisation node can be tested in stage simulation (without the need for robot).

        roscore
        rosrun map_server map_server <catkin_ws>/map.yaml
        rosrun stage_ros stageros <catkin_ws>/src/socspioneer/data/meeting.world
        roslaunch socspioneer keyboard_teleop.launch  # ---- run only if you want to move robot using keyboard
        rosrun pf_localisation node.py    # ----- requires laser_trace, and completed pf.py methods.

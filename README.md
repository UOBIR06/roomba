# roomba
Final project of Group 6 ([proposal](https://bham-my.sharepoint.com/:w:/r/personal/mxl367_student_bham_ac_uk/_layouts/15/Doc.aspx?sourcedoc=%7BCBCD51A3-804D-42BA-A23D-9412685EAAF5%7D&file=Intel.%20Robotics%20Project%20Proposal%20-%20Group%206.docx&action=edit&mobileredirect=true&wdPreviousSession=54702ad2-d520-4f52-a1b5-e6ba7baac8fe&wdOrigin=TEAMS-ELECTRON.p2p.undefined)), 
Intelligent Robotics 2021, UoB.

## Useful commands

Package setup and dependencies:
```bash
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

To [setup pycharm](https://youtu.be/lTew9mbXrAs?t=215) is you need, from 3:34.


To demo room exploration given `.png` map:
```bash
roslaunch roomba ipa_servers.launch
```

To demo frontier exploration on `meeting.world`:
```bash
roslaunch roomba explore.launch
rosrun roomba explore.py
```

To demo simple (non-MDP) sweeping:
```bash
roslaunch roomba clean.launch
rosrun roomba clean.py
```
## Practice
when publish/subscribe topics only used for this program. use `/roomba/` namespace to avoid conflicts.
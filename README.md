# VantTecSDVOrbslam
# VantTecSDVOrbslam
## Step 0 - Create a /orb_slam3_ros2_ws folder at home
This part of the code is hardcoded by the original creator of most of these packages, so the src folder must be inside of it, otherwise the code can't work, i tried to fix it but was unable to.

## Step 1 - Clean up the build folders 
Clean up any traces of my computer in the system so it doesn't think it is still in my computer, there was a script to delete all the builds folder but it was eaten by the git switch

## Step 2 - build the orbslam system
Remember to build the orbslam system before colcon building it ! 

```
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```
## Step 3 - Check for the path at CMakelists.txt
The path mustly works when the carpet is in /orb_slam3_ros2_ws, but if you don't want to do it that way or it still doesn't work it may be due to the path on CMakelists.txt, i recomend doing a symlink to fix this issue, if there is any way or you fix this issue please let me know.

The CmakeLists is in 
'''
cd src/ORB_SLAM3/

'''

## Step 4
Colcon build it !

```
cd ~/orb_slam3_ros2_ws
colcon build
source ~/orb_slam3_ros2_ws/install/setup.bash
```

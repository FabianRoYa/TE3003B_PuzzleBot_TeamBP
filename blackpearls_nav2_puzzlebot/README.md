# blackpearls_nav2_puzzlebot

**Description**  
A Python-based ROS 2 package that integrates the Nav2 navigation stack with the PuzzleBot simulation.


**Authors** 
- Fabian Erubiel Rojas Yañez - A01706636
- José Antonio Miranda Baños - A01611795
- Gaddiel Lara Roldán - A01704231
- Sergio Macías Corona - A01352038

**License**  
Apache 2.0 – see [LICENSE](https://github.com/FabianRoYa/TE3003B_PuzzleBot_TeamBP/blob/main/LICENSE)

---

**Requirements**
- Ubuntu 22.04
- ROS2 Humble
- Python3
- Gazebo Classic (6)

**Commands To Run**
```
# The command to run the mapping mode 
ros2 launch blackpearls_nav2_puzzlebot blackpearls_launch.py mode:=map
# The command to run the navigation modes
ros2 launch blackpearls_nav2_puzzlebot blackpearls_launch.py mode:=nav
```
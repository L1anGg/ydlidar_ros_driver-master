原为X3雷达驱动包

现临时修改为l1ang_planner规划包

   该路径规划结合了道格拉斯-普克(Douglas-Peuker)抽稀算法，让平滑的路径变为直线，
   使得底盘不太灵敏的差分小车能够正常导航,而不会因dwa等发布的平滑路径导致无法到达目标点
   源文件为l1ang_planner.py，由deepseek_v4.1flash转为c++程序可以直接从move_base启动
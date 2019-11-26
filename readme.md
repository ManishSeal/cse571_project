## Requirements
All packages installed on VM instance provided by AAIR Lab @ ASU.

## Running the Demo

To start the demo, run the following commands in order:

1. Run `roscore`
2. Run `rosrun group_11 server.py -sub 1 -b 1 -headless 0`[-sub--> number of subjects -b--> number of books -headless ---> 1 for headless mode or 0 for simulation in gazebo]
4. Run `roslaunch group_11 maze.launch`[launches environment specifies in step 1]
5. Run `rosrun group_11 move_tbot3.py`[launches services to make tbot move]
6. Run `rosrun group_11 book_spawner.py`[randomly spawns books in the environment]
7. Run `rosrun group_11 smart_book_keeper.py`[starts the smart book keeper robot]



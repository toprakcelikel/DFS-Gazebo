# DFS and Dijkstra-Gazebo and Ros2
<img width="612" height="408" alt="image" src="https://github.com/user-attachments/assets/3a2a48af-1678-4a19-8e59-1ddd16402b3d" />

I began by using the depth first search algorithm to allow the robot to find the target. The algorithm involves going as deep as possible in one direction before going back when reaching a dead end, going to another direction next. It follows this process until reaching the objective.

I added non-uniform objects to the map as well as electronics including lidar, color sensor, odometry, and imu to make robot movement more accurate and add functionality for it to interact with the environment.

After achieving using the DFS algorithm, I attempted to find a way to decrease the length of the robot's path until becoming the minimum length, which I was able to achieve using Dijkstra's algorithm. It attached weights to every coordinate, calculating the optimal next positions in advance to make sure it found the shortest path to the target.

The main three files I used to execute the integration was an sdf file, the algorithm file, and the turtlebot/environment generation file. The sdf file included the geometry of the environment and the specific objects, which formed the layout for the robot. The turtlebot/environment generation file being seperate was for ease of comprehension. 

I organized the algorithm file in such a way that I could effectively swap the DFS algorithm for the Dijkstra's algorithm by importing the main logic, helping make sure I didn't change any other parameters other than the algorithm itself.

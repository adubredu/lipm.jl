using Revise
using lipm

init_position = [0.0, 0.0, 0.0]
goal_position = [5.0, 0.0, 0.0]
X = move_to_position(init_position; goal_position=goal_position)
animate_walking_trajectory(X) 
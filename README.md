# lipm

[![Build Status](https://travis-ci.com/alphonsusadubredu/lipm.jl.svg?branch=main)](https://travis-ci.com/alphonsusadubredu/lipm.jl)
[![Build Status](https://ci.appveyor.com/api/projects/status/github/alphonsusadubredu/lipm.jl?svg=true)](https://ci.appveyor.com/project/alphonsusadubredu/lipm-jl)
[![Coverage](https://codecov.io/gh/alphonsusadubredu/lipm.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/alphonsusadubredu/lipm.jl)
[![Coverage](https://coveralls.io/repos/github/alphonsusadubredu/lipm.jl/badge.svg?branch=main)](https://coveralls.io/github/alphonsusadubredu/lipm.jl?branch=main)

This repository contains a Julia package that implements the 3D Linear Inverted Pendulum Mode walking model proposed by [Kajita et. al. IROS 2021](https://www.cs.cmu.edu/~hgeyer/Teaching/R16-899B/Papers/KajiitaEA01IEEE_ICIRS.pdf)


## Installation
`lipm` can be added via the Julia package manager.
Open a REPL and type `]`
Next type  `add https://github.com/alphonsusadubredu/lipm.jl` and press `Enter` to install the package and its dependencies.


## Usage
This example usage can be found at `examples/walk_to_linear_position.jl`

```
using lipm
init_position = [0.0, 0.0, 0.0]
goal_position = [5.0, 0.0, 0.0]
X = move_to_position(init_position; goal_position=goal_position)
animate_walking_trajectory(X) 

```
The simulation is rendered at a URL that looks like  `http://127.0.0.1:XXXX` in your REPL. 

Navigate to that URL in your browser to see the robot walk as shown below.

![](media/walking.gif)

This implementation is heavily inspired by [Chauby's Python implementation](https://github.com/chauby/BipedalWalkingRobots)
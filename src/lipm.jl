module lipm

using StaticArrays
using MeshCat
using GeometryBasics
using Colors
using CoordinateTransformations, Rotations
using LinearAlgebra

include("dynamics/walking_dynamics.jl")
include("dynamics/interface.jl")
include("visualization/animate.jl")


export 
    move_to_position,
    animate_walking_trajectory

end # module

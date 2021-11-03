using Revise
using lipm
using RigidBodyDynamics
using StaticArrays
using MeshCat, MeshCatMechanisms
using CoordinateTransformations

function jacobian_transpose_ik!(state::MechanismState,
    body::RigidBody,
    point::Point3D,
    desired::Point3D;
    α=0.1,
    iterations=100)
    mechanism = state.mechanism
    world = root_frame(mechanism)

    # Compute the joint path from world to our target body
    p = path(mechanism, root_body(mechanism), body)
    # Allocate the point jacobian (we'll update this in-place later)
    Jp = point_jacobian(state, p, transform(state, point, world))

    q = copy(configuration(state))

    for i in 1:iterations
        # Update the position of the point
        point_in_world = transform(state, point, world)
        # Update the point's jacobian
        point_jacobian!(Jp, state, p, point_in_world)
        # Compute an update in joint coordinates using the jacobian transpose
        Δq = α * Array(Jp)' * (transform(state, desired, world) - point_in_world).v
        # Apply the update
        q .= configuration(state) .+ Δq
        set_configuration!(state, q)
    end
    state
end

urdf = "examples/urdf/raphael.urdf"
mechanism = parse_urdf(urdf)
state = MechanismState(mechanism)

body = findbody(mechanism, "left_foot")
footpoint = Point3D(default_frame(body), 0.,0.,-0.)

mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf))
settransform!(mvis.visualizer, Translation(0,0,0.9))
# setelement!(mvis, footpoint, 0.01) 

# desired_location = Point3D(root_frame(mechanism), 0.1, 0., 0.)
# jacobian_transpose_ik!(state, body, footpoint, desired_location)
# set_configuration!(mvis, configuration(state))

qs = typeof(configuration(state))[]
for x in range(-0.2, stop=0.2, length=100)
    desired = Point3D(root_frame(mechanism), x, 0., -0.9)
    jacobian_transpose_ik!(state, body, footpoint, desired)
    push!(qs, copy(configuration(state)))
end
ts = collect(range(0, stop=2.5, length=length(qs)))
setanimation!(mvis, Animation(mvis, ts, qs))
render(mvis)
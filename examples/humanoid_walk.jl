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

lbody = findbody(mechanism, "left_foot")
lfootpoint = Point3D(default_frame(lbody), 0.,0.,-0.0)

rbody = findbody(mechanism, "right_foot")
rfootpoint = Point3D(default_frame(rbody), 0.,0.,-0.0)

mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf))
settransform!(mvis.visualizer, Translation(0,0,0.9))
# setelement!(mvis, footpoint, 0.01) 

# desired_location = Point3D(root_frame(mechanism), 0.5, 0., -0.9)
# jacobian_transpose_ik!(state, rbody, rfootpoint, desired_location)
# set_configuration!(mvis, configuration(state))
# println("leg at ", transform(state, rfootpoint, root_frame(mechanism)))

init_position = [0.0, 0.0, 0.0]
goal_position = [5.0, 0.0, 0.0]
x = move_to_position(init_position; goal_position=goal_position)

xb,yb,zb = x[1], x[2], ones(length(x[1]))
xl,yl,zl = x[3], x[4], x[5]
xr,yr,zr = x[6], x[7], x[8]

anim = Animation()
x = [0.3]
for i=1:3#length(xb)
    atframe(anim, i) do 
        settransform!(mvis.visualizer, Translation(x[1], 0, 0.9))
        ld = Point3D(root_frame(mechanism), x[1], -0.15, -0.9) 
        jacobian_transpose_ik!(state, lbody, lfootpoint, ld)
        set_configuration!(mvis, configuration(state) )
        rd = Point3D(root_frame(mechanism), x[1], 0.15, -0.9) 
        jacobian_transpose_ik!(state, rbody, rfootpoint, rd)
        set_configuration!(mvis, configuration(state) )
        x[1] = x[1].+0.1
    end
end
setanimation!(mvis, anim)


# init_position = [0.0, 0.0, 0.0]
# goal_position = [5.0, 0.0, 0.0]
# x = move_to_position(init_position; goal_position=goal_position)

# xb,yb,zb = x[1], x[2], ones(length(x[1]))
# xl,yl,zl = x[3], x[4], x[5]
# xr,yr,zr = x[6], x[7], x[8]

# anim = Animation()
# for i=1:length(xb)
#     atframe(anim, i) do 
#         settransform!(mvis.visualizer, Translation(xb[i], yb[i], 0.9))
#         ld = Point3D(root_frame(mechanism), xl[i], yl[i], -0.9)
#         println(ld)
#         jacobian_transpose_ik!(state, lbody, lfootpoint, ld)
#         set_configuration!(mvis, configuration(state) )
#         rd = Point3D(root_frame(mechanism), xr[i], yr[i], -0.9)
#         jacobian_transpose_ik!(state, rbody, rfootpoint, rd)
#         set_configuration!(mvis, configuration(state) )
#     end
# end
# setanimation!(mvis, anim)
# qs = typeof(configuration(state))[]
# for x in range(-0.2, stop=0.2, length=100)
#     desired = Point3D(root_frame(mechanism), x, 0., -0.9)
#     jacobian_transpose_ik!(state, body, footpoint, desired)
#     push!(qs, copy(configuration(state)))
# end
# ts = collect(range(0, stop=2.5, length=length(qs)))
# setanimation!(mvis, Animation(mvis, ts, qs))
render(mvis)
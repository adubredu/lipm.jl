function set_mesh!(vis)
    l,w,h = 0.4,0.4,1.0
    body = Sphere(Point3f0(0,0,0), 7l/16)
    setobject!(vis["robot"]["torso"]["body"], body, 
                            MeshPhongMaterial(color=colorant"yellow"))
    axle = Cylinder(Point3f0(0,0,0), Point3f0(0,w/2,0), 0.03f0)
    setobject!(vis["robot"]["torso"]["Laxle"], axle, 
                            MeshPhongMaterial(color=colorant"black"))
    setobject!(vis["robot"]["torso"]["Raxle"], axle, 
                            MeshPhongMaterial(color=colorant"black"))
    settransform!(vis["robot"]["torso"]["Laxle"], Translation(0,+l/4,0))
    settransform!(vis["robot"]["torso"]["Raxle"], Translation(0,-3l/4,0)) 

    foot = HyperSphere(Point3f0(0,0,0f0), 0.05f0) 
    Lleg = Cylinder(Point3f0(0,+l/2,0), Point3f0(0,+l/2,1), 0.03f0)
    Rleg = Cylinder(Point3f0(0,-l/2,0), Point3f0(0,-l/2,1), 0.03f0)
    setobject!(vis["robot"]["torso"]["Lleg"]["geom"], Lleg, 
                        MeshPhongMaterial(color=colorant=colorant"blue"))
    setobject!(vis["robot"]["torso"]["Rleg"]["geom"], Rleg, 
                        MeshPhongMaterial(color=colorant=colorant"green"))

    Lfoot = setobject!(vis["robot"]["torso"]["Lleg"]["geom"]["Lfoot"]["geom"], 
                    foot, MeshPhongMaterial(color=colorant"firebrick"))
    setobject!(vis["robot"]["torso"]["Rleg"]["geom"]["Rfoot"]["geom"], 
                    foot, MeshPhongMaterial(color=colorant"firebrick"))
    settransform!(vis["robot"]["torso"]["Lleg"]["geom"]["Lfoot"]["geom"], 
                    Translation(0,+l/2,h))
    settransform!(vis["robot"]["torso"]["Rleg"]["geom"]["Rfoot"]["geom"], 
                    Translation(0,-l/2,h))
return Lfoot
end

function animate_walk!(vis, x)
    xb,yb,zb = x[1], x[2], ones(length(x[1]))
    xl,yl,zl = x[3], x[4], x[5]
    xr,yr,zr = x[6], x[7], x[8]
    dt = 0.1
    anim = Animation()
    for i=1:length(xb)
        atframe(anim, i) do
            Llen = norm(SA[xl[i]-xb[i], zl[i]-zb[i]])
            Rlen = norm(SA[xr[i]-xb[i], zr[i]-zb[i]])
            θl = atan(xl[i]-xb[i], zl[i]-zb[i])
            θr = atan(xr[i]-xb[i], zr[i]-zb[i])  
            settransform!(vis["robot"]["torso"]["Lleg"], LinearMap(RotY(θl)))
            settransform!(vis["robot"]["torso"]["Rleg"], LinearMap(RotY(θr))) 
            settransform!(vis["robot"]["torso"], Translation(xb[i],yb[i],zb[i])) 
            settransform!(vis["robot"]["torso"]["Lleg"]["geom"],    
                                    LinearMap(Diagonal(SA[1,1,Llen])))
            settransform!(vis["robot"]["torso"]["Rleg"]["geom"], 
                                    LinearMap(Diagonal(SA[1,1,Rlen]))) 
        end
    end
    setanimation!(vis, anim)
end

function animate_walking_trajectory(X) 
    vis  = Visualizer()
    setprop!(vis["/Background"], "top_color", colorant"white")
    setprop!(vis["/Background"], "bottom_color", colorant"white")
    setprop!(vis["/Grid"], "visible",false)
    set_mesh!(vis)
    animate_walk!(vis, X)
    render(vis)
end

# X = move_to_position([0.0,0.0,1.0], goal_position=[5.0, 0.0, 1.0])
# animate_walking_trajectory(X)
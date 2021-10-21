mutable struct LIP
    dt::Float64
    t::Float64
    T_sup::Float64 #support time
    zc::Float64

    #desired foot location
    p_x::Float64
    p_y::Float64
    p_x_star::Float64 #modified foot location x
    p_y_star::Float64

    #Gait parameters
    s_x::Float64
    s_y::Float64

    #COM init conditions
    x_0::Float64
    vx_0::Float64
    y_0::Float64
    vy_0::Float64

    #COM real-time state
    x_t::Float64
    vx_t::Float64
    y_t::Float64
    vy_t::Float64

    #COM desired state
    x_d::Float64
    vx_d::Float64
    y_d::Float64
    vy_d::Float64

    #final state for one gait unit
    bar_x::Float64
    bar_vx::Float64
    bar_y::Float64
    bar_vy::Float64

    support_leg::String
    left_foot_pos::Array{Float64}
    right_foot_pos::Array{Float64}
    COM_pos::Array{Float64}
    T_c::Float64
    C::Float64
    S::Float64

    #physical params 
    m::Float64
    h::Float64

    function LIP(;dtt=0.001, Ts=1.0, sl="left_leg", mass=1.0, height=1.0)
        t = 0.0
        T_sup = Ts

        p_x=0
        p_y = 0.0
        p_x_star = 0.0 #modified foot location x
        p_y_star = 0.0
        support_leg = sl

        #Gait parameters
        s_y = 0.0
        s_x = 0.0

        #COM init conditions
        x_0 = 0.0
        vx_0 = 0.0
        y_0 = 0.0
        vy_0 = 0.0

        #COM real-time state
        x_t = 0.0
        vx_t = 0.0
        y_t = 0.0
        vy_t = 0.0
        zc = 0.0

        #COM desired state
        x_d = 0.0
        vx_d = 0.0
        y_d = 0.0
        vy_d = 0.0

        #final state for one gait unit
        bar_x = 0.0
        bar_vx = 0.0
        bar_y = 0.0
        bar_vy = 0.0

        left_foot_pos = SA[0.0,0.0,0.0]
        right_foot_pos = SA[0.0,0.0,0.0]
        COM_pos = SA[0.0,0.0,0.0]
        T_c = 0.0
        C = 0.0
        S = 0.0
        dt = dtt

        m = mass 
        h = height

        new(dt, t, T_sup, zc, p_x, p_y, p_x_star, p_y_star, s_x, s_y, x_0, vx_0,
        y_0, vy_0, x_t, vx_t, y_t, vy_t, x_d, vx_d, y_d, vy_d, bar_x, bar_vx,
        bar_y, bar_vy, support_leg, left_foot_pos, right_foot_pos, COM_pos, T_c,
        C, S, m, h)

    end
end


function initializeModel!(model, COM_pos, left_foot_pos, right_foot_pos)
        model.COM_pos = COM_pos
        model.left_foot_pos = left_foot_pos
        model.right_foot_pos = right_foot_pos

        model.zc = model.h
        model.T_c = sqrt(model.zc / 9.81) # set gravity parameter as 9.81
        model.C = cosh(model.T_sup/model.T_c)
        model.S = sinh(model.T_sup/model.T_c)
end

function updateParameters!(model, T_sup)
        model.T_sup = T_sup
        model.C = cosh(model.T_sup/model.T_c)
        model.S = sinh(model.T_sup/model.T_c)
end

function step!(model)
        model.t += model.dt
        t = model.t
        T_c = model.T_c

        model.x_t = model.x_0*cosh(t/T_c) + T_c*model.vx_0*sinh(t/T_c)
        model.vx_t = model.x_0/T_c*sinh(t/T_c) + model.vx_0*cosh(t/T_c)

        model.y_t = model.y_0*cosh(t/T_c) + T_c*model.vy_0*sinh(t/T_c)
        model.vy_t = model.y_0/T_c*sinh(t/T_c) + model.vy_0*cosh(t/T_c)
end

function calculateXtVt!(model, t)
        T_c = model.T_c 
        x_t = model.x_0*cosh(t/T_c) + T_c*model.vx_0*sinh(t/T_c)
        vx_t = model.x_0/T_c*sinh(t/T_c) + model.vx_0*cosh(t/T_c)

        y_t = model.y_0*cosh(t/T_c) + T_c*model.vy_0*sinh(t/T_c)
        vy_t = model.y_0/T_c*sinh(t/T_c) + model.vy_0*cosh(t/T_c) 

        return x_t, vx_t, y_t, vy_t
end

function nextReferenceFootLocation!(model, s_x, s_y, theta=0)
        if model.support_leg == "left_leg" # then the next support leg is the right leg
            p_x_new = model.p_x + cos(theta)*s_x - sin(theta)*s_y
            p_y_new = model.p_y + sin(theta)*s_x + cos(theta)*s_y
        elseif model.support_leg == "right_leg" # then the next support leg is the left leg
            p_x_new = model.p_x + cos(theta)*s_x + sin(theta)*s_y
            p_y_new = model.p_y + sin(theta)*s_x - cos(theta)*s_y
        end

        return p_x_new, p_y_new
end

function nextState!(model, s_x, s_y, theta=0.0)
        if model.support_leg == "left_leg"
            bar_x_new = cos(theta)*s_x/2.0 - sin(theta)*s_y/2.0
            bar_y_new = sin(theta)*s_x/2.0 + cos(theta)*s_y/2.0
        elseif model.support_leg == "right_leg"
            bar_x_new = cos(theta)*s_x/2.0 + sin(theta)*s_y/2.0
            bar_y_new = sin(theta)*s_x/2.0 - cos(theta)*s_y/2.0
        end
        return bar_x_new, bar_y_new
end

function nextVel!(model, bar_x=0, bar_y=0, theta=0)
        C = model.C
        S = model.S
        T_c = model.T_c

        bar_vx_new = cos(theta)*(1+C)/(T_c*S)*bar_x - 
                                sin(theta)*(C-1)/(T_c*S)*bar_y
        bar_vy_new = sin(theta)*(1+C)/(T_c*S)*bar_x + 
                                cos(theta)*(C-1)/(T_c*S)*bar_y

        return bar_vx_new, bar_vy_new
end

function targetState!(model, p_x, bar_x, bar_vx)
    x_d = p_x + bar_x
    vx_d = bar_vx

    return x_d, vx_d
end

function modifiedFootLocation!(model, a=1.0, b=1.0, x_d=0, vx_d=0, 
                                x_0=0, vx_0=0)
    C = model.C
    S = model.S
    T_c = model.T_c 
    D = a*(C - 1)^2 + b*(S/T_c)^2 
    p_x_star = -a*(C-1)*(x_d - C*x_0 - T_c*S*vx_0)/D - b*S*(vx_d - 
                                        S*x_0/T_c - C*vx_0)/(T_c*D) 
    return p_x_star
end

function calculateFootLocationForNextStep!(model, s_x=0.0, s_y=0.0, a=1.0,
                     b=1.0, theta=0.0, x_0=0.0, vx_0=0.0, y_0=0.0, vy_0=0.0)
    model.s_x = s_x
    model.s_y = s_y
    p_x_new, p_y_new = nextReferenceFootLocation!(model,s_x, s_y, theta) 
    bar_x, bar_y = nextState!(model,s_x, s_y, theta)
    bar_vx, bar_vy = nextVel!(model,bar_x, bar_y, theta) 

    model.x_d, model.vx_d = targetState!(model,p_x_new, bar_x, bar_vx)
    model.y_d, model.vy_d = targetState!(model,p_y_new, bar_y, bar_vy)  
    model.p_x_star = modifiedFootLocation!(model,a, b, model.x_d, 
                                            model.vx_d, x_0, vx_0)
    model.p_y_star = modifiedFootLocation!(model,a, b, model.y_d, 
                                            model.vy_d, y_0, vy_0) 
end

function switchSupportLeg!(model)
    if model.support_leg == "left_leg" 
        model.support_leg = "right_leg"
        COM_pos_x = model.x_t + model.left_foot_pos[1] 
        COM_pos_y = model.y_t + model.left_foot_pos[2]
        model.x_0 = COM_pos_x - model.right_foot_pos[1]
        model.y_0 = COM_pos_y - model.right_foot_pos[2]
    elseif model.support_leg == "right_leg" 
        model.support_leg = "left_leg"
        COM_pos_x = model.x_t + model.right_foot_pos[1]
        COM_pos_y = model.y_t + model.right_foot_pos[2]
        model.x_0 = COM_pos_x - model.left_foot_pos[1]
        model.y_0 = COM_pos_y - model.left_foot_pos[2]
    end

    model.t = 0
    model.vx_0 = model.vx_t
    model.vy_0 = model.vy_t
end

function at_goal(current_position, goal_position)
    if current_position[1] >= goal_position[1] 
        return true
    else
        return false
    end
end
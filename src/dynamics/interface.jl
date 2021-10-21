function move_to_position(init_position; init_velocity = [1.0, -0.01], 
        goal_position=[2.0,0.0,0.0], T=5.0, stride_length_x=0.5, 
        stride_length_y=0.4, a=1.0, b=1.0, delta_t=0.02)
    COM_pos_x = []
    COM_pos_y = []
    left_foot_pos_x = []
    left_foot_pos_y = []
    left_foot_pos_z = []
    right_foot_pos_x = []
    right_foot_pos_y = []
    right_foot_pos_z = []
    s_x = stride_length_x
    s_y = stride_length_y 
    theta = goal_position[3]

    # Initialize the COM position and velocity 
    COM_pos_0 =  init_position
    COM_v0 = init_velocity

    # Initialize the foot positions
    left_foot_pos = [COM_pos_0[1]+0.2, COM_pos_0[2]+0.1, 0 ]
    right_foot_pos = [COM_pos_0[1]+0.6, COM_pos_0[2]-0.5, 0]  

    LIPM_model = LIP(dtt=delta_t, Ts=0.5)
    initializeModel!(LIPM_model,COM_pos_0, left_foot_pos, right_foot_pos)

    LIPM_model.support_leg = "left_leg"  
    if LIPM_model.support_leg == "left_leg"
        support_foot_pos = LIPM_model.left_foot_pos
        LIPM_model.p_x = LIPM_model.left_foot_pos[1]
        LIPM_model.p_y = LIPM_model.left_foot_pos[2]
    else
        support_foot_pos = LIPM_model.right_foot_pos
        LIPM_model.p_x = LIPM_model.right_foot_pos[1]
        LIPM_model.p_y = LIPM_model.right_foot_pos[2]
    end

    LIPM_model.x_0 = LIPM_model.COM_pos[1] - support_foot_pos[1]
    LIPM_model.y_0 = LIPM_model.COM_pos[2] - support_foot_pos[2]
    LIPM_model.vx_0 = COM_v0[1]
    LIPM_model.vy_0 = COM_v0[2]  

    step_num = 0
    total_time = T # seconds
    global_time = 0

    swing_data_len = Int(LIPM_model.T_sup/delta_t)
    swing_foot_pos = zeros((swing_data_len, 3))
    j = 1

    switch_index = swing_data_len

    for i = 0:(Int(total_time/delta_t)-1)
        global_time += delta_t 
        step!(LIPM_model) 
        if step_num >= 1
            if LIPM_model.support_leg == "left_leg"
                LIPM_model.right_foot_pos = [swing_foot_pos[j,1],
                             swing_foot_pos[j,2], swing_foot_pos[j,3]]
            else
                LIPM_model.left_foot_pos = [swing_foot_pos[j,1], 
                            swing_foot_pos[j,2], swing_foot_pos[j,3]]

            end
            j += 1
        end

        # record data
        push!(COM_pos_x, LIPM_model.x_t + support_foot_pos[1])
        push!(COM_pos_y, LIPM_model.y_t + support_foot_pos[2])
        push!(left_foot_pos_x, LIPM_model.left_foot_pos[1])
        push!(left_foot_pos_y, LIPM_model.left_foot_pos[2])
        push!(left_foot_pos_z, LIPM_model.left_foot_pos[3])
        push!(right_foot_pos_x, LIPM_model.right_foot_pos[1])
        push!(right_foot_pos_y, LIPM_model.right_foot_pos[2])
        push!(right_foot_pos_z, LIPM_model.right_foot_pos[3])


        # switch the support leg
        if (i > 0) && (i % switch_index == 0)
            j = 1 
            switchSupportLeg!(LIPM_model) # switch the support leg  
            step_num += 1  
            if at_goal([COM_pos_x[end], COM_pos_y[end]], goal_position)  
                break 
            end 
            if LIPM_model.support_leg == "left_leg"
                support_foot_pos = LIPM_model.left_foot_pos
                LIPM_model.p_x = LIPM_model.left_foot_pos[1]
                LIPM_model.p_y = LIPM_model.left_foot_pos[2]
            else
                support_foot_pos = LIPM_model.right_foot_pos
                LIPM_model.p_x = LIPM_model.right_foot_pos[1]
                LIPM_model.p_y = LIPM_model.right_foot_pos[2]
            end 
            x_0, vx_0, y_0, vy_0 = calculateXtVt!(LIPM_model, LIPM_model.T_sup)  
            if LIPM_model.support_leg == "left_leg"
                x_0 = x_0 + LIPM_model.left_foot_pos[1]  
                y_0 = y_0 + LIPM_model.left_foot_pos[2]  
            else
                x_0 = x_0 + LIPM_model.right_foot_pos[1]  
                y_0 = y_0 + LIPM_model.right_foot_pos[2]  
            end
            
            calculateFootLocationForNextStep!(LIPM_model,s_x, s_y, a, b, theta, 
                                        x_0, vx_0, y_0, vy_0) 
 
            if LIPM_model.support_leg == "left_leg"
                right_foot_target_pos = [LIPM_model.p_x_star, 
                                        LIPM_model.p_y_star, 0]
                swing_foot_pos[:,1] = collect(range(LIPM_model.right_foot_pos[1]
                        , stop=right_foot_target_pos[1], length=swing_data_len))
                swing_foot_pos[:,2] = collect(range(LIPM_model.right_foot_pos[2]
                        , stop=right_foot_target_pos[2], length=swing_data_len))
                swing_foot_pos[2:swing_data_len, 3] .= 0.1
            else
                left_foot_target_pos = [LIPM_model.p_x_star, 
                                LIPM_model.p_y_star, 0] 
                swing_foot_pos[:,1] = collect(range(LIPM_model.left_foot_pos[1],
                         stop=left_foot_target_pos[1], length=swing_data_len))
                swing_foot_pos[:,2] = collect(range(LIPM_model.left_foot_pos[2],
                 stop=left_foot_target_pos[2], length=swing_data_len))
                swing_foot_pos[2:swing_data_len, 3] .= 0.1
            end 
        end
    end

    X = [COM_pos_x,
            COM_pos_y,
            left_foot_pos_x,
            left_foot_pos_y,
            left_foot_pos_z,
            right_foot_pos_x,
            right_foot_pos_y,
            right_foot_pos_z]
end
# Function for MC simulations
def run_simulation(
    q_rrt,
    q_astar,
    q_goal_rrt,
    q_goal_astar,
    q_goal_final,
    obstacle_speeds,
    rectangle_speeds,
    waypoints,
    obstacles_true,
    rect_obstacles,
    obstacles_noisy,
    expanded_rects,
    eps_expanded_rects,
    sigma,
    v_robot,
    path_data_rrt,
    path_data_astar,
    full_geometric_path_rrt,
    full_geometric_polyline_rrt,
    full_geometric_path_astar,
    full_geometric_polyline_astar,
    rrt,
    enable_animation=True
):
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from APF_RRT_Astar import init, update
    plt.close("all")

    fig, ax = plt.subplots(figsize=(8, 8), dpi = 50)
    ax.set_xlim(-50, 50)
    ax.set_ylim(-50, 50)
    ax.set_title("Dynamic Obstacle Avoidance Animation")
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    ax.grid(color='gray', linestyle='--', linewidth=0.4, alpha=0.7)

    # Raw paths (light)
    path_line_rrt,   = ax.plot([], [], color="#ffaaaa", linewidth=1, alpha=0.5, label="RRT raw")
    robot_dot_rrt,   = ax.plot([], [], 'o', color="#ffaaaa", markersize=5)
    path_line_astar, = ax.plot([], [], color="#aaaaff", linewidth=1, alpha=0.5, label="A* raw")
    robot_dot_astar, = ax.plot([], [], 'o', color="#aaaaff", markersize=5)
    # Smooth paths (bright)
    path_line_rrt_smooth,   = ax.plot([], [], color="red",  linewidth=2.5, label="RRT smooth")
    robot_dot_rrt_smooth,   = ax.plot([], [], 'ro', markersize=7)
    path_line_astar_smooth, = ax.plot([], [], color="blue", linewidth=2.5, label="A* smooth")
    robot_dot_astar_smooth, = ax.plot([], [], 'bo', markersize=7)
    
    true_scatter = ax.scatter([], [], c='black', s=40)
    noisy_scatter = ax.scatter([], [], c='red', s=40)
    goal_dot, = ax.plot(q_goal_final[0], q_goal_final[1], 'go', markersize=8, label="Goal")

    # Waypointleri mor yıldız olarak çiz
    ax.scatter(waypoints[:,0], waypoints[:,1],
    color='purple', marker='*', s=120, label="Stations")

    ax.legend(loc="upper right")

    # ── Speed slider ──────────────────────────────
    import numpy as _np
    import matplotlib.widgets as widgets
    fig.subplots_adjust(bottom=0.12)
    _base_speeds = obstacle_speeds.copy()
    _POOL_SIZE   = 60
    _n_orig      = len(obstacles_true)
    _pool_obs    = _np.zeros((_POOL_SIZE, 2))
    _pool_spd    = _np.zeros((_POOL_SIZE, 2))
    for _ii in range(_POOL_SIZE):
        _pool_obs[_ii] = obstacles_true[_ii % _n_orig] + _np.random.uniform(-5, 5, 2)
        _pool_spd[_ii] = _base_speeds[_ii % _n_orig]

    speed_slider_ax = fig.add_axes([0.15, 0.06, 0.70, 0.025])
    speed_slider = widgets.Slider(
        ax=speed_slider_ax, label="Pedestrian Speed",
        valmin=0.1, valmax=3.0, valinit=1.0,
        valstep=0.05, color="steelblue"
    )

    count_slider_ax = fig.add_axes([0.15, 0.02, 0.70, 0.025])
    count_slider = widgets.Slider(
        ax=count_slider_ax, label="# Pedestrians",
        valmin=1, valmax=_POOL_SIZE, valinit=_n_orig,
        valstep=1, color="tomato"
    )

    def _update_pedestrians():
        n   = int(count_slider.val)
        spd = speed_slider.val
        mystates["obstacles_true"]  = _pool_obs[:n].copy()
        mystates["obstacles_noisy"] = _pool_obs[:n].copy() + _np.random.normal(0, sigma, (n, 2))
        mystates["obstacle_speeds"] = (_pool_spd[:n] * spd).copy()
        mystates["n_active"]        = n

    speed_slider.on_changed(lambda val: _update_pedestrians())
    count_slider.on_changed(lambda val: _update_pedestrians())


    ellipse_patches         = []
    rect_patches            = []
    expanded_rect_patches   = []
    time                    = int(0)

    # ===============================
    # RUN ANIMATION
    # ===============================
    stop_counter_rrt      = 0
    stop_counter_astar    = 0

    ani             = None

    goals_achieved_so_far_rrt   = []
    goals_achieved_so_far_astar = []

    total_distance_rrt = 0
    collision_counter_rrt = 0
    total_distance_astar = 0
    collision_counter_astar = 0

    done = False
    done_rrt    = False
    done_astar  = False
    finish_counter_rrt   = 0
    finish_counter_astar = 0

    mystates = {
        "q_rrt": q_rrt,
        "q_astar": q_astar,
        "obstacle_speeds": obstacle_speeds,
        "ani": ani,
        "time": time,
        "q_goal_final": q_goal_final,
        "q_goal_rrt": q_goal_rrt,
        "q_goal_astar": q_goal_astar,
        "waypoints": waypoints,
        "obstacles_true": obstacles_true,
        "rectangle_speeds": rectangle_speeds,
        "rect_obstacles": rect_obstacles,
        "obstacles_noisy": obstacles_noisy,
        "sigma": sigma,
        "v_robot": v_robot,
        "path_data_rrt": path_data_rrt,
        "path_data_astar": path_data_astar,
        "path_line_rrt": path_line_rrt,
        "path_line_astar": path_line_astar,
        "robot_dot_rrt": robot_dot_rrt,
        "robot_dot_astar": robot_dot_astar,
        "true_scatter": true_scatter,
        "noisy_scatter": noisy_scatter,
        "goal_dot": goal_dot,
        "ellipse_patches": ellipse_patches,
        "rect_patches": rect_patches,
        "expanded_rect_patches": expanded_rect_patches,
        "ax": ax,
        "expanded_rects": expanded_rects,
        "eps_expanded_rects": eps_expanded_rects,
        "full_geometric_path_rrt": full_geometric_path_rrt,
        "full_geometric_path_astar": full_geometric_path_astar,
        "stop_counter_rrt": stop_counter_rrt,
        "stop_counter_astar": stop_counter_astar,
        "goals_achieved_so_far_rrt": goals_achieved_so_far_rrt,
        "goals_achieved_so_far_astar": goals_achieved_so_far_astar,
        "rrt": rrt,
        "full_geometric_polyline_rrt": full_geometric_polyline_rrt,
        "full_geometric_polyline_astar": full_geometric_polyline_astar,
        "total_distance_rrt": total_distance_rrt,
        "total_distance_astar": total_distance_astar,
        "collision_counter_rrt": collision_counter_rrt,
        "collision_counter_astar": collision_counter_astar,
        "done": done,
        "done_rrt": done_rrt,
        "done_astar": done_astar,
        "finish_counter_rrt": finish_counter_rrt,
        "finish_counter_astar": finish_counter_astar,
        "_pf_artists": [],
        "path_line_rrt_smooth":   path_line_rrt_smooth,
        "path_line_astar_smooth": path_line_astar_smooth,
        "robot_dot_rrt_smooth":   robot_dot_rrt_smooth,
        "robot_dot_astar_smooth": robot_dot_astar_smooth,
        "path_data_rrt_smooth":   [],
        "path_data_astar_smooth": [],
        "q_rrt_smooth":           q_rrt.copy(),
        "q_astar_smooth":         q_astar.copy(),
        "alpha_lpf":              0.5}

    # ── Potential field toggle button ─────────────
    from matplotlib.widgets import Button
    btn_ax = fig.add_axes([0.01, 0.92, 0.12, 0.06])
    pf_button = Button(btn_ax, "PField: OFF", color="lightgray", hovercolor="lightyellow")
    _pf_mode  = {"val": 0}
    _pf_labels = ["PField: OFF", "PField: HEAT", "PField: VEC", "PField: BOTH"]

    def on_pf_toggle(event):
        _pf_mode["val"] = (_pf_mode["val"] + 1) % 4
        pf_button.label.set_text(_pf_labels[_pf_mode["val"]])
        pf_button.ax.figure.canvas.draw_idle()

    pf_button.on_clicked(on_pf_toggle)
    mystates["pf_mode"]   = _pf_mode
    mystates["pf_button"] = pf_button

    def init_anim():
        return init(path_line_rrt, path_line_astar, robot_dot_rrt, robot_dot_astar, true_scatter, noisy_scatter, goal_dot)

    if enable_animation:
        ani = animation.FuncAnimation(
            fig,
            update,   # <-- PARANTEZ YOK
            frames=400,
            fargs=(mystates,),
            init_func=init_anim,
            interval=33,  # ~30 Hz target
            blit=False
        )

        mystates["ani"] = ani
        plt.show()
        
    
    else:
        frame = 0
        while not mystates["done"]:
            update(frame, mystates)
            frame += 1
        
    distance_rrt = mystates["total_distance_rrt"] 
    collisions_rrt = mystates["collision_counter_rrt"]
    cost_rrt = distance_rrt + 100 * collisions_rrt

    distance_astar = mystates["total_distance_astar"] 
    collisions_astar = mystates["collision_counter_astar"]
    cost_astar = distance_astar + 100 * collisions_astar
    
    print("FINAL DISTANCE RRT:", mystates["total_distance_rrt"])
    print("FINAL COLLISIONS RRT:", mystates["collision_counter_rrt"])
    print("FINAL COST RRT:", cost_rrt)

    print("FINAL DISTANCE ASTAR:", mystates["total_distance_astar"])
    print("FINAL COLLISIONS ASTAR:", mystates["collision_counter_astar"])
    print("FINAL COST ASTAR:", cost_astar)
    return distance_rrt, collisions_rrt, cost_rrt, distance_astar, collisions_astar, cost_astar
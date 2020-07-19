#PID Trajectory tracking

import math
import sys
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as la

dt = 0.01
Kp_lin = 10.0  # speed proportional gain
Kp_ang=5
Kp_track=1
show_animation = True
old_nearest_point_index = None

import cubic_spline_planner

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0,omega=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.omega=omega

def update(state,a_lin,a_ang):
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.omega*dt
    state.v = state.v + a_lin* dt
    state.omega = state.omega + a_ang*dt
    return state

def agent_state_update(state,v_in,omega_in):
    state.x = state.x + v_in * math.cos(state.yaw) * dt
    state.y = state.y + v_in * math.sin(state.yaw) * dt
    state.yaw = pi_2_pi(state.yaw + omega_in*dt)
    state.v = v_in
    state.omega = omega_in
    return state

def PIDControl(Kp,target,current):
    ac = Kp * (target - current)
    return ac

def PIDControl_track(Kp,error):
    error_tune=Kp*(error)
    return error_tune

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def calc_target_index(state, cx, cy):
    global old_nearest_point_index
    if old_nearest_point_index is None:
        dx = [state.x - icx for icx in cx]
        dy = [state.y - icy for icy in cy]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))
        old_nearest_point_index = ind
    else:
        ind = old_nearest_point_index
        distance_this_index = calc_distance(state, cx[ind], cy[ind])
        while True:
            ind = ind + 1 if (ind + 1) < len(cx) else ind
            distance_next_index = calc_distance(state, cx[ind], cy[ind])
            if distance_this_index < distance_next_index:
                break
            distance_this_index = distance_next_index
        old_nearest_point_index = ind

    return ind + 15

def calc_distance(state, point_x, point_y):
    dx = state.x - point_x
    dy = state.y - point_y
    return math.sqrt(dx ** 2 + dy ** 2)

def plot_arrow(x, y, yaw, length=1, width=2, fc="r", ec="k"):
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
         plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
            fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

def PIDcontroller(state,cx,cy,cyaw,ck,target_ind):
    
    #a_angular = - PIDControl(Kp_ang,target_angularspeed, state.omega)
    #a_linear = PIDControl(Kp_lin,target_linearspeed, state.v)
    
    yaw_new = (np.rad2deg(pi_2_pi(state.yaw)))
    if yaw_new < 0:
        yaw_new = yaw_new + 360
    dy = -state.y + cy[target_ind] 
    dx = -state.x + cx[target_ind]

    theta = np.rad2deg(pi_2_pi(math.atan2(dy, dx)))
    if theta < 0:
        theta = theta + 360

    error = (theta - yaw_new)
    if error > 180:
        error = error - 360
    if error <- 180:
        error = error + 360
    #Kp_track tune wisely
    state.omega = PIDControl_track(Kp_track,error)

    return state

def search_index(master_ind, master_state, cx, cy, gap):
    i,d = master_ind, 0
    while (d < 4):
        i = i - 1
        d = np.sqrt( np.square(master_state.x - cx[i]) + np.square(master_state.y - cy[i]))

    return i

def dist_error(L_state, F_state, targDist):
    dist = np.sqrt( np.square(L_state.x - F_state.x) + np.square(L_state.y - F_state.y))
    error = (targDist - dist)
    return error

def follower_desiredSpeed(cx, cy, prev_xx, prev_yy, ind):
    xx, yy = cx[ind], cy[ind]
    x_dot_d = (xx - prev_xx)/dt
    y_dot_d = (yy - prev_yy)/dt
    v_in = np.sqrt(np.square(x_dot_d) + np.square(y_dot_d))
    if v_in > 100:
        v_in = 0

    return v_in, xx, yy

def agent_linSpeed(v_d, L_error, F_error):
    v_in = v_d
    if np.absolute(L_error) > 1:
            v_in = v_d - (2*L_error)
    if np.absolute(F_error) > 1:
            v_in = v_d + (2*F_error)

    return v_in

def main():
    #ax = [0,10,20,30,40,50,60,70,80]  ##still creating few problem
    ax = [-10,20,30,50,60,30,0,-10]
    ay = [0,15,30,10,40,50,10,20]
    #ay = [math.sin(ix / 5.0) * ix / 2.0 for ix in ax]
    goal = [ax[-1], ay[-1]]
    cx, cy, cyaw, ck_curv, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
    ck = np.absolute(ck_curv) * [10]
    
    lastIndex = len(cx) - 1
    T, time, t = 100, 0.0, [0]
    gap = 6

    state1 = State(x=7, y=2, yaw=math.pi/3, v=0.0,omega=0.0)
    state2 = State(x=0, y=0, yaw=math.pi/3, v=0.0,omega=0.0)
    state3 = State(x=-8, y=0, yaw=math.pi/3, v=0.0,omega=0.0)
    state4 = State(x=-15, y=-5, yaw=math.pi/2, v=0.0,omega=0.0)
    target_ind1 = calc_target_index(state1, cx, cy)

    x1, y1, yaw1, v1, omega1 = [state1.x], [state1.y], [state1.yaw], [state1.v], [state1.omega]
    x2, y2, yaw2, v2, omega2 = [state2.x], [state2.y], [state2.yaw], [state2.v], [state2.omega]
    x3, y3, yaw3, v3, omega3 = [state3.x], [state3.y], [state3.yaw], [state3.v], [state3.omega]
    x4, y4, yaw4, v4, omega4 = [state4.x], [state4.y], [state4.yaw], [state4.v], [state4.omega]

    prev_xx2, prev_yy2 = state1.x, state2.y
    prev_xx3, prev_yy3 = state3.x, state3.y
    prev_xx4, prev_yy4 = state4.x, state4.y

    v_in1 = 50
    

    while T >= time and lastIndex> target_ind1 + 10:

        target_ind1 = calc_target_index(state1, cx, cy) 
        state1 = PIDcontroller(state1,cx,cy,cyaw,ck,target_ind1)
        state1 = agent_state_update(state1, v_in1, state1.omega)

        target_ind2 = search_index(target_ind1, state1, cx, cy, gap)
        target_ind3 = search_index(target_ind2, state2, cx, cy, gap)
        target_ind4 = search_index(target_ind3, state3, cx, cy, gap)
        #print(target_ind2, target_ind3)

        err12 = dist_error(state1, state2, gap)
        err23 = dist_error(state2, state3, gap)
        err34 = dist_error(state3, state4, gap)
        #print(err12, err23)

        v_d1 = 50
        v_d2, prev_xx2, prev_yy2 = follower_desiredSpeed(cx, cy, prev_xx2, prev_yy2, target_ind2)
        v_d3, prev_xx3, prev_yy3 = follower_desiredSpeed(cx, cy, prev_xx3, prev_yy3, target_ind3)
        v_d4, prev_xx4, prev_yy4 = follower_desiredSpeed(cx, cy, prev_xx4, prev_yy4, target_ind4)

        v_in1 = agent_linSpeed(v_d1, L_error=0, F_error=err12)
        v_in2 = agent_linSpeed(v_d2, L_error=err12, F_error=err23)
        v_in3 = agent_linSpeed(v_d1, L_error=err23, F_error=err34)
        v_in4 = agent_linSpeed(v_d1, L_error=err23, F_error=0)

        state2 = PIDcontroller(state2,cx,cy,cyaw,ck,target_ind2)
        state2 = agent_state_update(state2, v_in2, state2.omega)

        state3 = PIDcontroller(state3,cx,cy,cyaw,ck,target_ind3)
        state3 = agent_state_update(state3, v_in3, state3.omega)

        state4 = PIDcontroller(state4,cx,cy,cyaw,ck,target_ind4)
        state4 = agent_state_update(state4, v_in4, state4.omega)

        time = time + dt
        x1.append(state1.x)
        y1.append(state1.y)
        x2.append(state2.x)
        y2.append(state2.y)
        x3.append(state3.x)
        y3.append(state3.y)
        x4.append(state4.x)
        y4.append(state4.y)
        # yaw1.append(state1.yaw)
        # v1.append(state1.v)
        # omega1.append(state1.omega)
        # t.append(time)

        if show_animation:  # pragma: no cover
            plt.cla()
            plot_arrow(state1.x, state1.y, state1.yaw, fc="m")
            plot_arrow(state2.x, state2.y, state2.yaw, fc="g")
            plot_arrow(state3.x, state3.y, state3.yaw, fc="b")
            plot_arrow(state4.x, state4.y, state4.yaw, fc="c")

            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x1, y1, "-m", label="trajectory1")
            plt.plot(x2, y2, "-g", label="trajectory2")
            plt.plot(x3, y3, "-b", label="trajectory3")
            plt.plot(x4, y4, "-c", label="trajectory4")

            plt.plot(cx[target_ind1], cy[target_ind1], "x", label="target")
            plt.plot(cx[target_ind2], cy[target_ind2], "*", label="target")
            plt.plot(cx[target_ind3], cy[target_ind3], "+", label="target")
            plt.plot(cx[target_ind4], cy[target_ind4], "-", label="target")

            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state1.v)[:4])
            plt.pause(0.01) 
            #plt.pause(10)       

    assert lastIndex >= target_ind1, "Cannot goal"

if __name__ == '__main__':
    main()

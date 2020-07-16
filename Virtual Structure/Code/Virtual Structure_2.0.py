#attempt 1
#may have some several issue
#PID control trajectory tracking

import math
import sys
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as la

dt = 0.01
Kp_lin = 10.0  # speed proportional gain
Kp_ang=10
Kp_track=1
show_animation = True
old_nearest_point_index = None
sys.path.append("CubicSpline/")
try:
    import cubic_spline_planner
except ImportError:
    raise

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0,omega=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.omega=omega


def update(state,a_lin,a_ang):
    state.x=state.x + state.v * math.cos(state.yaw) * dt
    state.y=state.y + state.v * math.sin(state.yaw) * dt
    state.yaw=state.yaw + state.omega*dt
    state.v =state.v + a_lin* dt
    state.omega=state.omega + a_ang*dt
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
        # search nearest point index
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
    L = 0.0
    k = 0.03  # look forward gain
    Lfc = 0.2  # look-ahead distance
    Lf = k * state.v + Lfc
    # search look ahead target point index
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind] - state.x
        dy = cy[ind] - state.y
        L = math.sqrt(dx ** 2 + dy ** 2)
        ind += 1
    return ind

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

def PIDcontroller(state,cx,cy,cyaw,ck,target_ind,target_linearspeed,target_angularspeed):
    target_ind = calc_target_index(state, cx, cy) + 5 
    a_angular = -PIDControl(Kp_ang,target_angularspeed, state.omega)
    a_linear = PIDControl(Kp_lin,target_linearspeed, state.v)
    
    yaw_new=(np.rad2deg(pi_2_pi(state.yaw)))
    if yaw_new<0:
        yaw_new=yaw_new + 360
    dy=-state.y+cy[target_ind] 
    dx=-state.x+cx[target_ind]
    theta = pi_2_pi(math.atan2(dy, dx))
    theta=np.rad2deg(theta)
    if theta<0:
        theta=theta+360

    error=(theta-yaw_new)
    if error > 180:
        error=error-360
    if error <-180:
        error =error +360
    #Kp_track tune wisely
    state.omega=PIDControl_track(Kp_track,error)

    return state,target_ind,a_linear,a_angular

def desired_speed(xx, yy, prev_xx, prev_yy, prev_x_dot_d, prev_y_dot_d):
    #print("xx, prev_xx = ", '%.2f'%xx, '%.2f'%prev_xx)
    #print("yy, prev_yy = ", '%.2f'%yy, '%.2f'%prev_yy)
    x_dot_d = (xx - prev_xx)/dt
    y_dot_d = (yy - prev_yy)/dt
    #print(x_dot_d, y_dot_d)

    v_d = np.sqrt(np.square(x_dot_d) + np.square(y_dot_d))
    #print(v_d)
    
    #print(x_dot_d)#, prev_x_dot_d)
    #print(y_dot_d)#, prev_y_dot_d)
    x_dot_dot_d = (x_dot_d - prev_x_dot_d)/dt
    y_dot_dot_d = (y_dot_d - prev_y_dot_d)/dt
    #print(x_dot_dot_d, y_dot_dot_d)
    #print(prev_x_dot_d, prev_y_dot_d)

    if x_dot_d == 0 and y_dot_d == 0:
        omega_d = 0

    else:
        omega_d = ((y_dot_dot_d * x_dot_d) - (x_dot_dot_d * y_dot_d))/ \
              (np.square(x_dot_d) + np.square(y_dot_d))

    if omega_d > 4:
        omega_d = 4
        print(omega_d)
    if omega_d < -4:
        omega_d = -4
        print(omega_d)
 
    #print("---------")

    return v_d, omega_d, xx, yy, x_dot_d, y_dot_d

def error_claculation(xx_d, yy_d, state, state_n):

    xe = (xx_d - state_n.x)*np.cos(state_n.yaw) + \
               (yy_d - state_n.y)*np.sin(state_n.yaw)
    ye = -(xx_d - state_n.x)*np.sin(state_n.yaw) + \
               (yy_d - state_n.y)*np.cos(state_n.yaw)
    yawe = pi_2_pi(state.yaw - state_n.yaw)

    return xe, ye, yawe

def target_coordinate(state, p):
    xx_d1 = state.x + p * np.cos(pi_2_pi(state.yaw))
    yy_d1 = state.y + p * np.sin(pi_2_pi(state.yaw))
    xx_d2 = state.x + p * np.cos(pi_2_pi(state.yaw + np.deg2rad(120)))
    yy_d2 = state.y + p * np.sin(pi_2_pi(state.yaw + np.deg2rad(120)))
    xx_d3 = state.x + p * np.cos(pi_2_pi(state.yaw + np.deg2rad(240)))
    yy_d3 = state.y + p * np.sin(pi_2_pi(state.yaw + np.deg2rad(240)))
    return xx_d1, yy_d1, xx_d2, yy_d2, xx_d3, yy_d3

def target_coordinate_new(state, px1, py1, px2, py2, px3, py3):
    state.yaw = pi_2_pi(state.yaw)
    xx_d1 = state.x + (px1 * np.cos(state.yaw)) - (py1 * np.sin(state.yaw))
    yy_d1 = state.y + (px1 * np.sin(state.yaw)) + (py1 * np.cos(state.yaw))
    xx_d2 = state.x + (px2 * np.cos(state.yaw)) - (py2 * np.sin(state.yaw))
    yy_d2 = state.y + (px2 * np.sin(state.yaw)) + (py2 * np.cos(state.yaw))
    xx_d3 = state.x + (px3 * np.cos(state.yaw)) - (py3 * np.sin(state.yaw))
    yy_d3 = state.y + (px3 * np.sin(state.yaw)) + (py3 * np.cos(state.yaw))
    return xx_d1, yy_d1, xx_d2, yy_d2, xx_d3, yy_d3


def main():

    ax = [0,20,40,60,80,100]
    ay = [0,0,10,20,20,15]
    goal = [ax[-1], ay[-1]]

    cx, cy, cyaw, ck_curv, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
    ck = np.absolute(ck_curv)*[10]
    T = 100.0  # max simulation time

    state = State(x=0.0, y=0.0, yaw=0, v=0.0,omega=0.0)
    lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    omega=[state.omega]
    t = [0.0]
    target_ind = calc_target_index(state, cx, cy)

    p = 4
    px1, py1 = 4, 4 
    px2, py2 = -4, 4
    px3, py3 = 0, -4

    xx_d1, yy_d1, xx_d2, yy_d2, xx_d3, yy_d3 = target_coordinate_new(state, px1, py1, px2, py2, px3, py3)

    state_1 = State(x= xx_d1, y=yy_d1, yaw=0, v=0.0,omega=0.0)
    state_2 = State(x= xx_d2, y=yy_d2, yaw=0, v=0.0,omega=0.0)
    state_3 = State(x= xx_d3, y=yy_d3, yaw=0, v=0.0,omega=0.0)

    target_angularspeed = 0
    target_linearspeed = 120 / 3.6

    prev_xx1, prev_yy1 = xx_d1, yy_d1
    prev_x_dot_d1, prev_y_dot_d1 = 0, 0
    #x_dot_d1, y_dot_d1 = 0, 0

    prev_xx2, prev_yy2 = xx_d2, yy_d2
    prev_x_dot_d2, prev_y_dot_d2 = 0, 0
    #x_dot_d2, y_dot_d2 = 0, 0

    prev_xx3, prev_yy3 = xx_d3, yy_d3
    prev_x_dot_d3, prev_y_dot_d3 = 0, 0
    #x_dot_d3, y_dot_d3 = 0, 0

    cx1, cy1, cyaw1 = 1, 1, 5
    cx2, cy2, cyaw2 = 1, 1, 5
    cx3, cy3, cyaw3 = 1, 1, 5

    cx12, cy12, cyaw12 = 1,1,5
    cx13, cy13, cyaw13 = 1,1,5

    cx21, cy21, cyaw21 = 1,1,5
    cx23, cy23, cyaw23 = 1,1,5

    cx31, cy31, cyaw31 = 1,1,5
    cx32, cy32, cyaw32 = 1,1,5

    while T >= time and lastIndex > target_ind + 10:

        xx_d1, yy_d1, xx_d2, yy_d2, xx_d3, yy_d3 = target_coordinate_new(state, px1, py1, px2, py2, px3, py3)
        print("------")

        v_d1, omega_d1, xx_d1, yy_d1, x_dot_d1, y_dot_d1 = \
        desired_speed(xx_d1, yy_d1, prev_xx1, prev_yy1, prev_x_dot_d1, prev_y_dot_d1)
        prev_xx1 = xx_d1
        prev_yy1 = yy_d1
        prev_x_dot_d1 = x_dot_d1
        prev_y_dot_d1 = y_dot_d1
        #print(v_d1*3.6, omega_d1*3.6)


        v_d2, omega_d2, xx_d2, yy_d2, x_dot_d2, y_dot_d2 = \
        desired_speed(xx_d2, yy_d2, prev_xx2, prev_yy2, prev_x_dot_d2, prev_y_dot_d2)
        prev_xx2 = xx_d2
        prev_yy2 = yy_d2
        prev_x_dot_d2 = x_dot_d2
        prev_y_dot_d2 = y_dot_d2
        #print(v_d2*3.6, omega_d2*3.6)


        v_d3, omega_d3, xx_d3, yy_d3, x_dot_d3, y_dot_d3 = \
        desired_speed(xx_d3, yy_d3, prev_xx3, prev_yy3, prev_x_dot_d3, prev_y_dot_d3)
        prev_xx3 = xx_d3
        prev_yy3 = yy_d3
        prev_x_dot_d3 = x_dot_d3
        prev_y_dot_d3 = y_dot_d3
        #print(v_d3*3.6, omega_d3*3.6)

        xe1, ye1, yawe1 = error_claculation(xx_d1, yy_d1, state, state_1)
        #print(xe1, ye1, yawe1)

        xe2, ye2, yawe2 = error_claculation(xx_d2, yy_d2, state, state_2)
        #print(xe2, ye2, yawe2)

        xe3, ye3, yawe3 = error_claculation(xx_d3, yy_d3, state, state_3)
        #print(xe3, ye3, yawe3)


        v_in1 = v_d1 + (cx1*xe1) - (cy1*omega_d1*ye1) + (cx12*(xe1 - xe2)) + (cx13*(xe1 - xe3)) - ((cy12*omega_d1*(ye1 - ye2)) + (cy13*omega_d1*(ye1 - ye3)) )
        omega_in1 = omega_d1 + (cyaw1*yawe1) + (cyaw12*(yawe1 - yawe2)) + (cyaw13*(yawe1 - yawe3)) 

        v_in2 = v_d2 + (cx2*xe2) - (cy2*omega_d2*ye2) + \
                (cx21*(xe2 - xe1)) + (cx23*(xe2 - xe3)) - \
                ((cy21*omega_d2*(ye2 - ye1)) + (cy23*omega_d2*(ye2 - ye3)))
        omega_in2 = omega_d2 + (cyaw2*yawe2) + \
                    (cyaw21*(yawe2 - yawe1)) + (cyaw23*(yawe2 - yawe3))

        v_in3 = v_d3 + (cx3*xe3) - (cy3*omega_d3*ye3) + \
                (cx31*(xe3 - xe1)) + (cx32*(xe3 - xe2)) - \
                ((cy31*omega_d3*(ye3 - ye1) + cy32*omega_d3*(ye3 - ye2)))
        omega_in3 = omega_d3 + (cyaw3*yawe3) + \
                    (cyaw31*(yawe3 - yawe1)) + (cyaw32*(yawe3 - yawe2) )

        # print(v_in1, omega_in1) 
        # print(v_in2, omega_in2)
        # print(v_in3, omega_in3)    

        state_1 = agent_state_update(state_1, v_in1, omega_in1)
        state_2 = agent_state_update(state_2, v_in2, omega_in2)
        state_3 = agent_state_update(state_3, v_in3, omega_in3)

        state,target_ind,a_linear,a_angular= PIDcontroller(state,cx,cy,cyaw,ck,target_ind,target_linearspeed,target_angularspeed)
        state = update(state,a_linear,a_angular)
        time = time + dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        omega.append(state.omega)
        t.append(time)

        if show_animation:  # pragma: no cover
            plt.cla()
            # plot_arrow(state.x, state.y, state.yaw, fc="r")
            # plot_arrow(xx_d1, yy_d1, state.yaw,fc="c")
            # plot_arrow(xx_d2, yy_d2, state.yaw,fc="y")
            # plot_arrow(xx_d3, yy_d3, state.yaw,fc="g")

            plot_arrow(state_1.x, state_1.y, state_1.yaw,fc="c")
            plot_arrow(state_2.x, state_2.y, state_2.yaw,fc="y")
            plot_arrow(state_3.x, state_3.y, state_3.yaw,fc="g")
            
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4] + "  Angular Speed:" + str(state.omega)[:4])
            plt.pause(0.001)    
            #plt.pause(1)    

    assert lastIndex >= target_ind, "Cannot goal"
#

        #plt.show()

if __name__ == '__main__':
    main()


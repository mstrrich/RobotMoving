
import matplotlib.pyplot as plt
import math
import numpy as np
import random
import imageio
import os

def generate_wall(num, h, w):
    walls = []
    while(len(walls) < num):
        px = random.uniform(0, w)
        py = random.uniform(0, h)
        # ori = random.uniform(0, math.pi)
        ori = 0
        if([px, py, ori] not in walls):
            walls.append([px, py, ori])
    return [h, w, walls]

def draw_environment(ax, env, wall_len):
    h, w, walls = env
    ax.set_ylim([0, h])
    ax.set_xlim([0, w])
    point_x = []
    point_y = []
    for i in range(len(walls)):
        px = walls[i][0]
        py = walls[i][1]
        ori = walls[i][2]
        
        # draw line
        ax.plot([px- math.cos(ori)* wall_len/2, px+ math.cos(ori)* wall_len/2], 
                [py- math.sin(ori)* wall_len/2, py+ math.sin(ori)* wall_len/2], 'b-', lw=5)
        
        # center point of wall
        point_x.append(px)
        point_y.append(py)
    
    ax.scatter(point_x, point_y, color='red', linewidths=4)

def draw_walls(env, wall_len):
    h, w, walls = env
    plt.ylim([0, h])
    plt.xlim([0, w])
    point_x = []
    point_y = []
    for i in range(len(walls)):
        px = walls[i][0]
        py = walls[i][1]
        ori = walls[i][2]
        
        # draw line
        plt.plot([px- math.cos(ori)* wall_len/2, px+ math.cos(ori)* wall_len/2], 
                [py- math.sin(ori)* wall_len/2, py+ math.sin(ori)* wall_len/2], 'b-', lw=5)
        
        # center point of wall
        point_x.append(px)
        point_y.append(py)
    
    plt.scatter(point_x, point_y, color='red', linewidths=4)

def radian_check(r):
    while(r>2* math.pi):
        r = r - 2* math.pi
    while(r<0):
        r = r + 2* math.pi
    else:
        pass
    return r

def Move(robot):
    rx = robot[0]
    ry = robot[1]
    r_ori = robot[2]
    stddev_m = 0.1
    stddev_a = 0.01
    
    # motion command
    command_m = 0
    angle = 0
    distance = 0
    
    # Move: robot
    count = 0
    while(True):
        # command_m = random.uniform(0, np.sqrt(9**2 + 3**2)) # maximum distance is the diagonal of the boundary
        command_m = random.uniform(0, 1)
        # turn
        angle = math.radians(random.normalvariate(mu=0, sigma=command_m* stddev_a))
        
        # move forward
        distance = random.normalvariate(mu=command_m, sigma=stddev_m* command_m)
        
        # check boundary condition
        new_rx = rx + distance* math.cos(r_ori + angle)
        new_ry = ry + distance* math.sin(r_ori + angle)
        
        if(new_rx<=9 and new_rx>=0 and new_ry<=3 and new_ry>=0):
            robot[0] = new_rx
            robot[1] = new_ry
            robot[2] = radian_check(r_ori + angle)
            # print('robot move: distance={}, angle={}'.format(distance, angle))
            break
        else:
            if(count < 5):
                count = count + 1
            else:
                command_m = 0
                angle = 0
                distance = 0
                # print('robot move: distance={}, angle={}'.format(distance, angle))
                break
    
    
    
                
        
def Turn(robot):
    r_ori = robot[2]
    stddev_a = 0.01
    
    # Turn: robot
    command_t = random.uniform(0, 360)
    angle = math.radians(random.normalvariate(mu=command_t, sigma=command_t* stddev_a))
    # update orientation
    robot[2] = radian_check(r_ori + angle)
    # print("Turn: dr={}".format(angle))
    


def Motion_model(robot):
    command = random.randint(0, 3)
    
    if(command==0):
        # print('Command0:')
        Move(robot)
    elif(command==1):
        # print('Command1:')
        Turn(robot)
    elif(command==2):
        # print('Command2:')
        Move(robot)
        Turn(robot)
    else:
        # print('Command3:')
        Turn(robot)
        Move(robot)

def cal_distance(a, b):
    return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def help_sensor(particle, wall, d, sigma_d, sigma_angle, sigma_orientation):
    dist = random.normalvariate(mu=d, sigma=d* sigma_d)
    angle = radian_check(random.normalvariate(mu=np.arctan(wall[1]/wall[0]+0.001)-np.arctan(particle[1]/particle[0]+0.001)
                                              , sigma=d* sigma_angle))
    orientation = radian_check(random.normalvariate(mu=particle[2]-wall[2], sigma=d* sigma_orientation))
    return dist, angle, orientation

def Sensor_model(particle, walls):
    # check whether in the visual domain
    walls_in_domain = []
    for i in range(len(walls)):
        # relative cooridinate to particle
        w_x = walls[i][0] - particle[0]
        w_y = walls[i][1] - particle[1]
        w_ori = np.arctan(w_y/w_x)
        if(w_x>0 and w_y>0): 
            pass
        elif(w_x<0 and w_y>0): 
            w_ori = w_ori + math.radians(180)
        elif(w_x<0 and w_y<0): 
            w_ori = w_ori + math.radians(180)
        elif(w_x>0 and w_y<0): 
            pass
        else:
            pass
        
        # determine
        if(abs(w_ori-particle[2])<math.radians(40)):
            walls_in_domain.append(walls[i])
        
    # detect or not detect
    walls_detect = []
    for i in range(len(walls_in_domain)):
        d = cal_distance([particle[0], particle[1]], [walls_in_domain[i][0], walls_in_domain[i][1]])
        
        # case1: d>=1
        if(d >= 1):
            if(random.random() <= 0.1):
                dist, angle, orientation = help_sensor(particle, walls_in_domain[i], d, 0.1, 0.05, 0.3)
                walls_detect.append([dist, angle, orientation])
            else:
                if(random.random() < 0.05): #0.95
                    dist = random.uniform(1, 3)
                    angle = random.uniform(-math.radians(40), math.radians(40)) + particle[2]
                    orientation = random.uniform(0, 2* math.pi)
                    walls_detect.append([dist, angle, orientation])
        # case2: 0.5<=d<1
        elif(d >= 0.5):
            if(random.random() <= 0.8):
                dist, angle, orientation = help_sensor(particle, walls_in_domain[i], d, 0.01, 0.01, 0.1)
                walls_detect.append([dist, angle, orientation])
            else:
                if(random.random() < 0.1): #0.9
                    dist = random.uniform(0.5, 1)
                    angle = random.uniform(-math.radians(40), math.radians(40)) + particle[2]
                    orientation = random.uniform(0, 2* math.pi)
                    walls_detect.append([dist, angle, orientation])
        
        # case3: 0<=d<0.5
        else:
            if(random.random() <= 0.6):
                dist, angle, orientation = help_sensor(particle, walls_in_domain[i], d, 0.05, 0.1, 10)
                walls_detect.append([dist, angle, orientation])
            else:
                if(random.random() < 0.01): #0.99
                    dist = random.uniform(0,0.5)
                    angle = random.uniform(-math.radians(40), math.radians(40)) + particle[2]
                    orientation = random.uniform(0, 2* math.pi)
                    walls_detect.append([dist, angle, orientation])
                    
    return walls_detect

            
def moving_robot(N_obstacle, iteration):
    ''' generate environment '''
    # initialize
    h = 3
    w = 9
    wall_len = 0.5
    
    # generate environment
    env = generate_wall(N_obstacle, h, w)
    
    
    ''' particle filter '''
    # Initialize
    walls = env[2]
    
    # robot position
    robot = [9/2, 3/2, 0]

    # start iterations
    filenames = []
    for i in range(iteration):
        print("iteration: {}......".format(i+1))
        
        # robot move and particle gets the same command and movements
        while(True):
            Motion_model(robot, )
            r_detect = Sensor_model(robot, walls)
            if(len(r_detect)>0):
                break
            else:
                pass
    
        # calculate particle weight by robot's sensing
        # save picture for animation
        draw_walls(env, wall_len)
        plt.title('iteration: {}'.format(i))
        plt.xlabel('x')
        plt.ylabel('y')
        plt.scatter(robot[0], robot[1], color='green', linewidths=15)  
        plt.scatter(robot[0]+0.1*np.cos(robot[2]), robot[1]+0.1*np.sin(robot[2]), color='black', linewidths=1)
        plt.savefig('moving_robot{}.png'.format(i))
        filenames.append('moving_robot{}.png'.format(i))
        plt.close()
    
    ''' visialization '''
    # animation
    with imageio.get_writer('animation/moving_robot_with {} obstacle.gif'.format(N_obstacle), mode='I', duration=0.5) as writer:
        for i in range(1, len(filenames)):
            image = imageio.imread(filenames[i])
            writer.append_data(image)
    
    for filename in set(filenames):
        os.remove(filename)
        
    
    # # final iteration
    # fig = plt.figure( dpi = 200 )
    # ax = fig.add_subplot(1, 1, 1)
    # draw_environment(ax, env, wall_len)
    
    # p_x_head = []
    # p_y_head = []
    # for i in range(len(p_x)):
    #     p_x_head.append(p_x[i]+0.05*np.cos(p_ori[i]))
    #     p_y_head.append(p_y[i]+0.05*np.sin(p_ori[i]))
    
    # # draw particles
    # ax.scatter(p_x_head[0 : int(-N_particle*0.1)], p_y_head[0 : int(-N_particle*0.1)], color='black', linewidths=0.1)
    
    # # draw robot
    # ax.scatter(robot[0], robot[1], color='green', linewidths=10)  
    # ax.scatter(p_x[0 : int(-N_particle*0.1)], p_y[0 : int(-N_particle*0.1)], color='yellow', linewidths=5)
    # ax.scatter(robot[0]+0.05*np.cos(robot[2]), robot[1]+0.05*np.sin(robot[2]), color='black', linewidths=0.1)
    
    


        

#!/usr/bin/python3

# Obstacle avoidance using Vector Polar Histogam
# [VPH: A New Laser Radar Based Obstacle Avoidance Method for Intelligent Mobile Robots]
import numpy as np
import matplotlib.pyplot as plt


class Point:
    def __init__(self, angle, range, x, y) -> None:
        self.angle = angle
        self.range = range
        self.x = x
        self.y = y
        self.cost = float('inf')

def calDist(x1,y1,x2,y2):
    return np.sqrt((y2-y1)**2 + (x2-x1)**2)

if __name__ == '__main__':
    #Load file
    data = []
    file = "C:\\Users\\ruby\\Desktop\\YDLidar-SDK-master_hi\\build\\Debug\\scan_pts1.txt"
    with open(file, "r") as f:
        for line in f.readlines():
            line_split = line.split(",")
            line_split[-1] = line_split[-1].replace("\n", "")
            data.append([float(x) for x in line_split])
            
    original_data = data
    robot_step = []
    colors = plt.get_cmap('tab20')
    
    sample_rate = 5
    dt = 1/sample_rate
    sample_gap = 2*np.pi/(3000/sample_rate)
    k1, k2 = 1.5, 1
    target_pos = initial_target_pos = [0,1]
    target_dir = np.arctan2(target_pos[0], target_pos[1]) if target_pos[0] != 0 else np.sign(target_pos[1])*np.pi/2
    robot_dir = np.pi/2
    robot_pos = [0,0]
    robot_vel = 0.1
    robot_dec = 0.05
    robot_radius = 0.05
    gap_distance = safety_distance = 1.1*robot_radius
    robot_face = np.pi/2
    
    binary_polar_histogram = []
    last_window_i = 0
    min_cost, min_cost_i, min_cost_window_index = float('inf'), None, [None, None]
    window_i =0
    cnt = 0
    
    while calDist(robot_pos[0],robot_pos[1], target_pos[0], target_pos[1]) > 0.01 and cnt < 30:
        robot_step.append(robot_pos.copy())
        s1 = robot_vel*robot_vel/(2*robot_dec) # Decelerating space component
        for row in data:
            theta, dis, intensity = row
            angle_diff = abs(theta-robot_face) if abs(theta-robot_face) < np.pi else abs(abs(theta-robot_face)-np.pi)
            D = s1*(np.cos(angle_diff)**2) + safety_distance if (angle_diff <= np.pi/2) else 0 # Distance threshold

            xx, yy = D*np.cos(theta), D*np.sin(theta)
            plt.scatter(xx, yy, s=1, color='red')
            # if (abs(angle_diff) < 0.2 and dis < 5): 
            #     x,y = dis*np.cos(theta), dis*np.sin(theta)
            #     print((x,y), theta, theta*180/np.pi, dis, D)

            if (dis < 5):
                plt.scatter(x, y, s=3, color='black')

            x, y = dis*np.cos(theta), dis*np.sin(theta) 
            if (dis > D):
                binary_polar_histogram.append(Point(theta, dis, x, y))
            elif last_window_i != len(binary_polar_histogram):
                p1 = binary_polar_histogram[last_window_i]
                p2 = binary_polar_histogram[-1]
                a1,a2,r1,r2 = p1.angle, p2.angle, p1.range, p2.range
                width = np.sqrt(r2**2+r1**2-2*r1*r2*np.cos(a2-a1))
                if (width > gap_distance):
                    # print("Window: ", window_i, (last_window_i, len(binary_polar_histogram)),  width, gap_distance)
                    # print(a1, r1, (r1*np.cos(a1), r1*np.sin(a1)))
                    # print(a2, r2, (r2*np.cos(a2), r2*np.sin(a2)))
                    for i in range(last_window_i, len(binary_polar_histogram)):
                        p = binary_polar_histogram[i]
                        target_diff = abs(p.angle - target_dir) if abs(p.angle-target_dir) < np.pi else abs(abs(p.angle-target_dir)-np.pi)
                        robot_diff = abs(p.angle - robot_face) if abs(p.angle-robot_face) < np.pi else abs(abs(p.angle-robot_face)-np.pi)
                        p.cost = k1*target_diff + k2*robot_diff 
                        if p.cost < min_cost:
                            min_cost = p.cost
                            min_cost_i = i
                            min_cost_window_index = [last_window_i, len(binary_polar_histogram)]

                        if (p.range < 5):
                            x,y = p.x, p.y
                            plt.scatter(x, y, s=7, marker="_", color=colors(window_i%20))
                        
                    
                last_window_i = len(binary_polar_histogram)
                window_i+=1




        if not min_cost_i:
            a,r = np.pi/2, dt*robot_vel
            x,y = 0, r
        else:
            # min_gap_point_cnt_half = int(np.ceil(np.arctan2(gap_distance/2, r)/sample_gap))
            min_gap_point_cnt_half = 10
            temp = None
            if (min_cost_i - min_cost_window_index[0] > min_gap_point_cnt_half and min_cost_window_index[1] - min_cost_i > min_gap_point_cnt_half):
                i = min_cost_i
                temp = 'A'
            elif (min_cost_i - min_cost_window_index[0] > min_gap_point_cnt_half):
                i = min_cost_window_index[1] - min_gap_point_cnt_half*2
                temp = 'B'
            else:
                i = min_cost_window_index[0] + min_gap_point_cnt_half*2
                temp = 'C'
            p = binary_polar_histogram[i]
            a,r = p.angle, p.range
            x,y= p.x, p.y
            print(temp, i, min_cost_i,  binary_polar_histogram[min_cost_i].range, p.range)
        print(binary_polar_histogram[min_cost_i].x, binary_polar_histogram[min_cost_i].y, binary_polar_histogram[min_cost_i].angle, p.angle, p.range)
        print(f"{cnt}, i: {i}, min_cost: {min_cost}, min_point: {(x,y)}, robot_pos: {robot_pos}")

        plt.plot([0, x/r], [0,y/r], "-.", color='green', linewidth='1')
        plt.plot([-gap_distance/2, -gap_distance/2], [-0.15, 0.15], '--', [gap_distance/2, gap_distance/2], [-0.15, 0.15], '--' ,color='blue')
        plt.scatter(target_pos[0], target_pos[1], color='green', marker='x')
        plt.scatter(0, 0, s=20, color='red', marker='x')
    
        # dr, da = dt*robot_vel, a - robot_face
        # dx, dy = dr*np.cos(a), dr*np.sin(a)
        # robot_pos[0] += dx
        # robot_pos[1] += dy
        # target_pos[0] -= dx
        # target_pos[1] -= dy
        # target_dir = np.arctan2(target_pos[0], target_pos[1]) + np.pi/2 if target_pos[0] != 0 else np.sign(target_pos[1])*np.pi/2
        # target_dir =  target_dir if abs(target_dir) <= np.pi else np.sign(target_dir)*-1*(2*np.pi-abs(target_dir))
        # new_data = []
        # for row in data:
        #     theta, dis, intensity = row
        #     a1 = theta - da
        #     a1 =  a1 if abs(a1) <= np.pi else np.sign(a1)*-1*(2*np.pi-abs(a1))
        #     r1 = dis - dr
        #     new_data.append((a1,r1, intensity))
        #     x,y = r1*np.cos(a1), r1*np.sin(a1)
        #     if (dis < 5):
        #         plt.scatter(x, y, s=3, color='blue')

        plt.show()
        data = new_data
        last_window_i = 0
        binary_polar_histogram = []
        min_cost, min_cost_i, min_cost_window_index = float('inf'), None, [None, None]
        cnt+=1
    
    for row in original_data:
        theta, dis, intensity = row
        if (dis < 5):
            x,y = dis*np.cos(theta), dis*np.sin(theta)
            plt.scatter(x, y, s=5, color='black')
    plt.scatter(initial_target_pos[0], initial_target_pos[1], color='green', marker='x')
    for step in robot_step:
        plt.scatter(step[0], step[1], s=3, color='red', marker='x')
    plt.show()

    print("HI")
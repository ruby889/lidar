import matplotlib.pyplot as plt
import math
import os
import shutil
if __name__ == "__main__":
    path = './image'
    shutil.rmtree(path)
    os.makedirs(path)

    filename = "./build/log.txt"
    f = open(filename, "r")
    points, obstacles = [], []
    image_i = 0
    for line in f:
        data = line.split("|")
        
        #lidar points
        pts = data[0].strip().split()
        for p in pts:
            x,y = map(float, p.strip('()').split(','))
            points.append([x,y])
        
        #obstacles
        blocks = data[1].strip().split("/")
        for bk in blocks:
            bk_pts = []
            pts = bk.strip().split()
            for p in pts:
                x,y = map(float, p.strip('()').split(','))
                bk_pts.append([x,y])
            obstacles.append(bk_pts)

        safety_distance, s1, robot_width = map(float, data[2].split(","))
        robot_x, robot_y, robot_angle    = map(float, data[3].split(","))
        target_x, target_y, target_angle = map(float, data[4].split(","))
        final_pt_angle                   = map(float, data[5])
        
        for x,y in points:
            plt.plot(x,y,1.0, c="black")
        
        for i in range(len(obstacles)):
            color = i / range(len(obstacles))
            for x,y in obstacles[i]:
                plt.plot(x, y,1.0, c=color, cmap="tab20")

        for theta in range(-180, 180):
            rad = theta*math.pi/180
            angle_diff = abs(rad - robot_angle)
            D = s1*math.pow(math.cos(angle_diff),2) + safety_distance if (angle_diff <= math.pi/2) else 0
            x,y = D*math.cos(theta), D*math.sin(theta)
            plt.plot(x,y,c="red")
        
        plt.plot([robot_width/2, robot_width/2, -robot_width/2, -robot_width/2, robot_width/2],
                  [-robot_width/2, robot_width/2, robot_width/2, -robot_width/2, -robot_width/2])
        plt.quiver([robot_x], [robot_y], [math.cos(robot_angle)], [math.sin(robot_angle)])
        plt.scatter([target_x],[target_y], 15, marker='X', c='red')
        plt.plot([0, 0.5*math.cos(final_pt_angle)], [0, 0.5*math.sin(final_pt_angle)])
        plt.savefig(f'./image/{image_i}.png')
        image_i += 1
    



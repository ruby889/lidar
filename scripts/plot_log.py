import matplotlib
import matplotlib.pyplot as plt
import math
import os
import shutil
import numpy as np
if __name__ == "__main__":
    path = './image'
    if (os.path.exists(path)):
        shutil.rmtree(path)
    os.makedirs(path)

    filename = "./../build/log.txt"
    num_lines = sum(1 for _ in open(filename))
    image_i = 1
    plt.figure(figsize=(10,7))
    for line in open(filename, "r"):
        print(f"Processing image {image_i}/{num_lines}.")
        plt.clf()
        points, obstacles,obstacles_colors = [], [], []
        data = line.split("|")
        
        #lidar points
        pts = data[0].strip().split()
        for p in pts:
            x,y = map(float, p.strip('()').split(','))
            points.append([x,y])
        
        #obstacles
        blocks = data[1].strip().split("/")
        for i in range(len(blocks)):
            bk = blocks[i]
            pts = bk.strip().split()
            bk_pts = []
            for p in pts:
                x,y = map(float, p.strip('()').split(','))
                bk_pts.append([x,y])
            obstacles.extend(bk_pts)
            obstacles_colors.extend([i/len(obstacles)]*len(bk_pts))

        safety_distance, s1, robot_width = map(float, data[2].split(","))
        robot_x, robot_y, robot_angle    = map(float, data[3].split(","))
        target_x, target_y, target_angle = map(float, data[4].split(","))
        final_pt_angle                   = float(data[5])
        
        #plot lidar points
        X,Y = zip(*points)
        plt.scatter(X,Y,s=1.0, c="black")

        #plot obstacles
        cmap = matplotlib.cm.get_cmap('tab20')
        colors = obstacles_colors
        X,Y =  zip(*obstacles)
        plt.scatter(X,Y, s=3.0, c=colors, cmap='tab20')

        #plot safety boundaries
        X,Y = [],[]
        for theta in range(-180, 180):
            rad = theta*math.pi/180
            angle_diff = abs(rad - robot_angle)
            D = s1*math.pow(math.cos(angle_diff),2) + safety_distance if (angle_diff <= math.pi/2) else 0
            X.append(D*math.cos(rad))
            Y.append(D*math.sin(rad))
        plt.scatter(X,Y, s=1.0, c="red")
        
        #plot others
        plt.plot([robot_width/2, robot_width/2, -robot_width/2, -robot_width/2, robot_width/2],
                  [-robot_width/2, robot_width/2, robot_width/2, -robot_width/2, -robot_width/2])
        plt.quiver([robot_x], [robot_y], [math.cos(robot_angle)], [math.sin(robot_angle)])
        plt.scatter([target_x],[target_y], 15, marker='X', c='red')
        plt.plot([0, 0.5*math.cos(final_pt_angle)], [0, 0.5*math.sin(final_pt_angle)])
        plt.savefig(f'./image/{image_i}.png')
        image_i += 1
    print("Done.")



#!/usr/bin/python3

# Have custering, half finished
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from scipy.ndimage import rotate

class Point:
    def __init__(self) -> None:
        self.obj_group: int
        self.group_id: int = 0
        self.angle: float
        self.range: float
        self.x: float
        self.y: float
        self.prev: Point = None
        self.next: Point = None

class Group:
    def __init__(self) -> None:
        self.id: int = 0
        self.point_cnt: int = 0
        self.first_pt: Point
        self.last_pt: Point
        self.type = None

class Circle:
    def __init__(self) -> None:
        self.x: float = 0.0
        self.y: float = 0.0
        self.r: float = 0.0

class Line:
    def __init__(self) -> None:
        self.p1 = None
        self.p2 = None

class Rectangle:
    def __init__(self) -> None:
        self.p1: float
        self.p2: float
        self.p3: float
        self.p4: float

def calDist(n1, n2):
    return np.sqrt((n2.y-n1.y)**2 + (n2.x-n1.x)**2)

def calDist1(x1,y1,x2,y2):
    return np.sqrt((y2-y1)**2 + (x2-x1)**2)

def line_line_intersation(m1, b1, m2, b2):
    x = (b2 - b1)/(m1 - m2)
    y = m1 * x + b1
    return x,y

def minimum_bounding_rectangle(points):
    """
    Find the smallest bounding rectangle for a set of points.
    Returns a set of points representing the corners of the bounding box.

    :param points: an nx2 matrix of coordinates
    :rval: an nx2 matrix of coordinates
    """
    
    pi2 = np.pi/2.

    # get the convex hull for the points
    hull_points = points[ConvexHull(points).vertices]

    # calculate edge angles
    edges = np.zeros((len(hull_points)-1, 2))
    edges = hull_points[1:] - hull_points[:-1]

    angles = np.zeros((len(edges)))
    angles = np.arctan2(edges[:, 1], edges[:, 0])

    angles = np.abs(np.mod(angles, pi2))
    angles = np.unique(angles)

    # find rotation matrices
    # XXX both work
    rotations = np.vstack([
        np.cos(angles),
        np.cos(angles-pi2),
        np.cos(angles+pi2),
        np.cos(angles)]).T
#     rotations = np.vstack([
#         np.cos(angles),
#         -np.sin(angles),
#         np.sin(angles),
#         np.cos(angles)]).T
    rotations = rotations.reshape((-1, 2, 2))

    # apply rotations to the hull
    rot_points = np.dot(rotations, hull_points.T)

    # find the bounding points
    min_x = np.nanmin(rot_points[:, 0], axis=1)
    max_x = np.nanmax(rot_points[:, 0], axis=1)
    min_y = np.nanmin(rot_points[:, 1], axis=1)
    max_y = np.nanmax(rot_points[:, 1], axis=1)

    # find the box with the best area
    areas = (max_x - min_x) * (max_y - min_y)
    best_idx = np.argmin(areas)

    # return the best box
    x1 = max_x[best_idx]
    x2 = min_x[best_idx]
    y1 = max_y[best_idx]
    y2 = min_y[best_idx]
    r = rotations[best_idx]

    p1 = np.dot([x1, y2], r)
    p2 = np.dot([x2, y2], r)
    p3 = np.dot([x2, y1], r)
    p4 = np.dot([x1, y1], r)

    return p1, p2, p3, p4

def clustering(prev_group, prev):
    prev_group.last_pt = prev
    if (prev_group.point_cnt <= 5):
        c = prev_group.type = Circle()
        middle_pt, temp = prev, prev
        max_dis = -1
        for i in range(prev_group.point_cnt//2): middle_pt = prev.prev
        for i in range(prev_group.point_cnt): 
            c.r = max(c.r, calDist(middle_pt, temp))
            temp = temp.prev
        c.x, c.y = middle_pt.x, middle_pt.y
        draw = plt.Circle((c.x, c.y), c.r, color='r', fill=False)
        plt.gca().add_patch(draw)
    else:
        p, q = prev_group.first_pt, prev_group.last_pt
        m = (p.y - q.y)/(p.x - q.x)
        b = p.y - m * p.x
        s = calDist(p, q)
        temp = prev
        Dmax = -1
        for i in range(prev_group.point_cnt):
            Dmax = max(Dmax, abs(m*temp.x - temp.y + b)/np.sqrt(m*m + 1))
            temp = temp.prev

        if (Dmax < 0.2*abs(s)):
            prev_group.type = Line()
            prev_group.type.p1 = p
            prev_group.type.p2 = q
            plt.plot([p.x, q.x],[p.y, q.y], color='r', linewidth=1.0)
        else:
            prev_group.type = Rectangle()
            pts = []
            temp = prev
            for i in range(prev_group.point_cnt):
                pts.append([temp.x, temp.y])
                temp = temp.prev
            p1, p2, p3, p4 = minimum_bounding_rectangle(np.array(pts))
            prev_group.type.p1 = p1
            prev_group.type.p2 = p2
            prev_group.type.p4 = p4
            prev_group.type.p3 = p3
            plt.plot([p1[0], p2[0], p3[0], p4[0], p1[0]],[p1[1], p2[1], p3[1], p4[1], p1[1]], color='r', linewidth = 1.0)

def lie_outside_rectange(p1, p2, p3, p4, p):
    edge_pts = [p1, p2, p3, p4, p1]
    for i in range(4):
        x1, y1 = edge_pts[i]
        x2, y2 = edge_pts[i+1]
        xp, yp = p
        d = (x2 - x1) * (yp - y1) - (xp - x1) * (y2 - y1)
        return True if d < 0 else False


if __name__ == '__main__':
    data = []
    file = "C:\\Users\\ruby\\Desktop\\YDLidar-SDK-master_hi\\build\\Debug\\scan_pts.txt"
    with open(file, "r") as f:
        for line in f.readlines():
            line_split = line.split(",")
            line_split[-1] = line_split[-1].replace("\n", "")
            data.append([float(x) for x in line_split])
    
    colors = plt.get_cmap('tab20')
    theshold = 0.5
    start, cnt = 0, 0
    groups = []
    head = prev = Point()
    
    robot_dir = 0
    robot_pos = (0,0)
    robot_w = 0.05
    gap_w = 1.1*robot_w
    action_space = 0.15
    robot_angle = 0
    target = (0,1)
    type_cnt = [0]
    check_range = 0.15

    obj_centers = []
    while start < len(data):
        #Robot pos
        plt.scatter(0, 0, s=20, color='red', marker='x')
        group_pos_sum = np.array([0, 0])
        blocked = False
        blocked_group = -1
        
        for row in data[start:]:
            theta, dis, intensity = row

            #For one circle
            if (cnt > 100 and theta <= -2.8): break
            if (dis > 15): continue
            # theta = 2*np.pi + theta #From -PI/PI to 0/2PI
            cnt += 1

            current = Point()
            current.angle = theta
            current.range = dis
            current.x = dis*np.cos(theta)
            current.y = dis*np.sin(theta)
            current.prev = prev
            prev.next = current

            #Cluster points into group
            if not prev.prev or calDist(prev,current) > gap_w:
            # if (not prev or calDist1(gx,gy, current.x, current.y) > 0.06*grange*prev_group.point_cnt):
                group = Group()
                group.id = len(groups)
                group.first_pt = current
                current.obj_group = group.id
                current.group_id = 0
                if prev.prev:
                    clustering(groups[-1], prev)
                group_pos_sum = np.array([current.x, current.y])
                group.point_cnt += 1
                groups.append(group)
            else:
                current.obj_group = prev.obj_group
                current.group_id = prev.group_id + 1
                groups[-1].point_cnt += 1
                group_pos_sum += [current.x, current.y]
            if not blocked:
                blocked = (abs(current.x) < robot_w/2 and current.y < check_range)
                blocked_group = groups[current.obj_group]
            prev = current
            plt.scatter(current.x, current.y, s=3, color=colors(current.obj_group%20))
        prev_group = groups[-1]
        prev_group.last_pt = current
        prev_group.x, prev_group.y = group_pos_sum/prev_group.point_cnt
        prev_group.range = np.sqrt(prev_group.x**2+prev_group.y**2)
        current.next = head.next
        current.next.prev = current
        clustering(groups[-1], prev)

        # alpha = np.arctan((target[1] - robot_pos[1]) / (target[0] - robot_pos[0]))
        # direction = alpha - robot_dir
        if blocked:
            print("Blocked")
            if type(blocked_group.type) == Circle:
                x,y,r = -blocked_group.type.x, -blocked_group.type.y, blocked_group.type.r
                d = np.sqrt(x*x+y*y)
                P00, P01 = np.array([x,y]), np.array([-y, x])
                P1 = (r*r/d*d)*P00 + (r/d*d)*np.sqrt(d*d-r*r)*P01
                P2 = (r*r/d*d)*P00 - (r/d*d)*np.sqrt(d*d-r*r)*P01
                plt.plot(P1[0], P1[1], 'bx')
                plt.plot(P2[0], P2[1], 'bx')
                angle1, angle2 = np.arctan(P1[1]/P1[0]), np.arctan(P2[1]/P2[0])
                q = P1 if angle1 < angle2 else P2                
                plt.plot(q[0], q[1], 'rx')

            elif type(blocked_group.type) == Line:
                p1, p2 = blocked_group.type.p1, blocked_group.type.p2
                q = p1.prev if (p1.angle < p2.angle) else p2.next
                x, y = (p1.x + q.x)/2, (p1.y+q.y)/2
                plt.scatter(x,y, s=5, color='red')
                plt.plot([p1.x, q.x], [p1.y, q.y], color='green')
            else:
                p1, p2, p3, p4 = blocked_group.type.p1, blocked_group.type.p2, blocked_group.type.p3, blocked_group.type.p4
                min_pt, min_angle = None, 360
                for p in [p1, p2, p4, p4]:
                    angle = np.arctan(p[1]/p[0])
                    u =  (p[0] - robot_pos[0])/np.sqrt(p[0]**2 + robot_pos[0]**2), (p[1] - robot_pos[1])/np.sqrt(p[1]**2 + robot_pos[1]**2)
                    if angle < min_angle and lie_outside_rectange(p1, p2, p3, p4, p): 
                        min_pt = p
                        min_angle = angle
                plt.plot(p[0], p[1], 'rx')


        start += cnt
        cnt = 0
        plt.plot([-robot_w/2, -robot_w/2], [-0.15, 0.15], '--', [robot_w/2, robot_w/2], [-0.15, 0.15], '--' ,color='blue')
        plt.scatter(target[0], target[1], color='green', marker='x')
        plt.show()
    print("HI")
    
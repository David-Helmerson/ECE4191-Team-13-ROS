import math
import numpy as np


class CardioidPlanner:
    """
    Planning returns a point on a cardioid closest to the goal point
    """
    def __init__(self, radius, n_points):
        # Points on cardioid are given in polar coordinates
        self.points = [(2*radius*(1 - math.cos(th)), th) 
                        for th in np.linspace(-math.pi, math.pi, n_points, endpoint=False)]
        self.left = False
        self.right = False
    
    def plan(self, pose, goal):
        # Find cardioid point closest to goal, left and right sensor measurements permitting
        min_dist2, min_point = math.inf, None
        for p in self.points:
            if (p[1] <= 0 and self.left) or (p[1] >= 0 and self.right):
                # Find euclidean distance from cardioid point to goal
                p_dist2 = (p[0]*math.cos(pose[2]+p[1]) + pose[0] - goal[0])**2 + \
                         (p[0]*math.sin(pose[2]+p[1]) + pose[0] - goal[0])**2 
                if p_dist2 < min_dist2: min_dist2, min_point = p_dist2, p

        return min_point[0]*math.cos(min_point[1]), min_point[0]*math.sin(min_point[1])


class TentaclePlanner:
    
    def __init__(self, dt=0.1, steps=5, alpha=1, beta=0.0):
        
        self.dt = dt
        self.steps = steps
        # Tentacles are possible trajectories to follow
        self.tentacles = [(0.0,1.0),(0.0,-1.0),(0.1,1.0),(0.1,-1.0),(0.1,0.5),(0.1,-0.5),(0.1,0.0),(0.0,0.0)]
        
        self.alpha = alpha
        self.beta = beta
        
        # Status of obstacles detected to the left and right
        self.left = False 
        self.right = False
    
    # Play a trajectory and evaluate where you'd end up
    def roll_out(self,v,w,goal_x,goal_y,goal_th,x,y,th):
        
        for j in range(self.steps):
        
            x = x + self.dt*v*np.cos(th)
            y = y + self.dt*v*np.sin(th)
            th = (th + w*self.dt)
            
            if (w > 0 and self.left) or (w < 0 and self.right):
                return np.inf
        
        # Wrap angle error -pi,pi
        e_th = goal_th-th
        e_th = np.arctan2(np.sin(e_th),np.cos(e_th))
        
        cost = self.alpha*((goal_x-x)**2 + (goal_y-y)**2) + self.beta*(e_th**2)
        
        return cost
        
    
    # Choose trajectory that will get you closest to the goal
    def plan(self,goal_x,goal_y,goal_th,x,y,th):
        
        costs = []
        for v,w in self.tentacles:
            costs.append(self.roll_out(v,w,goal_x,goal_y,goal_th,x,y,th))
        
        best_idx = np.argmin(costs)
        
        return self.tentacles[best_idx]



def pp_waypoint(path, pos, dist, max_lookahead=math.inf):
    """
    Obtains the first point along a path a specified distance from a 
    specified position, and the last path waypoint before the found point. 
    Returns None if no such point can be found.

    Parameters
    ----------
    path : iterable
        xy coordinates of path waypoints
    pos : iterable
        xy coordinate of current robot position
    dist : float
        lookahead distance for pure pursuit algorithm
    max_lookahead : int
        the maximum number of waypoints that will be scanned ahead for a
        suitable point

    Returns
    -------
    tuple
        xy coordinate of found point along path
    None
        When no such xy coordinate can be found
    """

    x1, y1 = None, None
    # Consider each line segment between waypoints
    for i, x2, y2 in enumerate(path):
        if x1 is not None:
            # Determine wether line intersects circle around pos
            x12, y12, x23, y23 = x1-x2, y1-y2, x2-pos[0], y2-pos[1]
            det = dist*(x12**2 + y12**2) - (x12*y23 - y12*x23)**2
            if det > 0:
                # If so, determine wether line segment does
                a, b, det = x12**2+y12**2, x12*x23+y12*y23, math.sqrt(det)
                t1, t2 = (b+det)/a, (b-det)/a
                if (0 <= t1 and t1 <= 1) or (0 <= t2 and t2 <= 1):
                    # If so, determine which intersection is earlier and return
                    t = t1 if t1 > t2 or t2 < 0 or 1 < t2 else t2
                    return (t*x1+(1-t)*x2, t*y1+(1-t)*y2), i
        
        if i > max_lookahead: return None, None  # Max lookahead surpassed
        x1, y1 = x2, y2
    # No point found
    return None, None
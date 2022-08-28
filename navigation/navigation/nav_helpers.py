import math

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
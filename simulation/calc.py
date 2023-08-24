import math, circle

turning_pts_right = [
    [-0.574, 2.354],
    [-2.26, 2.895],
    [-1.715, 4.386],
    [-0.269, 4.024]
]
turning_pts_left = [
    [-0.592, 2.93],
    [-1.707, 2.908],
    [-1.718, 4.047],
    [-0.612, 4.04]
]
right = [
    [-0.179, 2.929],
    [-1.715, 2.512],
    [-2.132, 4.022],
    [-0.609, 4.434]
]
left = [
    [-1.713, 4.05],
    [-0.607, 4.053],
    [-0.57, 2.9],
    [-1.702, 2.893]
]

left_circle, right_circle = circle.circle_lst()
border = [(-1.7,1.06), (-3.39,4.015), (-0.62,5.675), (0.7, 2.93)]

merging_pts = {
    "straight_A": [(-0.579, 2.94), (-0.604, 4.035), (-0.608, 4.447)],
    "right_A": [(-0.228, 2.93)],
    "left_A": [(-0.579, 2.94), (-1.078, 3.468), (-1.73, 4.05), (-2.14, 4.034)],
    "straight_B": [(-1.72, 2.91), (-0.579, 2.94), (-0.228, 2.93)],
    "right_B": [(-1.72, 2.516)],
    "left_B": [(-1.72, 2.91), (-1.078, 3.468), (-0.604, 4.035), (-0.608, 4.447)],
    "straight_C": [(-1.73, 4.05), (-1.72, 2.91), (-1.72, 2.516)],
    "right_C": [(-2.14, 4.034)],
    "left_C": [(-1.73, 4.05), (-1.078, 3.468), (-0.579, 2.94), (-0.228, 2.93)],
    "straight_D": [(-0.604, 4.035), (-1.73, 4.05), (-2.14, 4.034)],
    "right_D": [(-0.608, 4.447)],
    "left_D": [(-0.604, 4.035), (-1.078, 3.468), (-1.72, 2.91), (-1.72, 2.516)]
}

def merging_dst(pos, path):
    x,y = pos
    index = ord(path[-1])-65
    if "straight" in path:
        if len(merging_pts[path]) != 0:
            d = math.sqrt((x-merging_pts[path][0][0])**2 + (y-merging_pts[path][0][1])**2)
            if d <= 0.01:
                merging_pts[path].pop(0)
        else:
            d=0
    elif "right" in path:
        if len(merging_pts[path]) != 0:
            if ("A" in path and y < turning_pts_right[0][1]) or ("C" in path and y > turning_pts_right[2][1]):
                d = abs(y-turning_pts_right[index][1]) + circle.arc_length(right_circle[index], turning_pts_right[index], merging_pts[path][0])
            elif ("B" in path and x < turning_pts_right[1][0]) or ("D" in path and x > turning_pts_right[3][0]):
                d = abs(x-turning_pts_right[index][0]) + circle.arc_length(right_circle[index], turning_pts_right[index], merging_pts[path][0])
            else:
                if ("A" in path and x < right[0][0]) or ("C" in path and x > right[2][0]) or ("B" in path and y > right[1][1]) or ("D" in path and y < right[3][1]):
                    d = circle.arc_length(right_circle[index], (x,y), merging_pts[path][0])
                else:
                    d = 0

            if d <= 0.01:
                merging_pts[path].pop()
        else:
            d = 0
    elif "left" in path:
        if len(merging_pts[path]) != 0:
            if ("A" in path and y < turning_pts_left[0][1]) or ("C" in path and y > turning_pts_left[2][1]):
                d = abs(y-merging_pts[path][0][1])
            elif ("B" in path and x < turning_pts_right[1][0]) or ("D" in path and x > turning_pts_right[3][0]):
                d = abs(x-merging_pts[path][0][0])
            elif ("A" in path and x < left[0][0]) or ("C" in path and x > left[2][0]):
                d = abs(x-merging_pts[path][0][0])
            elif ("B" in path and y > left[1][1]) or ("D" in path and y < left[3][1]):
                d = abs(y-merging_pts[path][0][1])
            else:
                d = circle.arc_length(left_circle[index], (x,y), merging_pts[path][0])

            if d <= 0.01:
                merging_pts[path].pop(0)
        else:
            d=0
    return d

def signed_dist(pos, path, stop_d = 0.2):
    x,y = pos
    stop = False
    pid = ["straight", "right", "left"]
    # try:
    d = merging_dst(pos, path)
    # except ValueError:
    #     d = 1e9

    if path == "straight_A":
        if abs(y-border[2][1]) <= stop_d:
            stop = True
        return x - turning_pts_right[0][0], d, stop, pid[0]
    elif path == "straight_B":
        if abs(x-border[3][0]) <= stop_d:
            stop = True
        return -(y - turning_pts_right[1][1]), d, stop, pid[0]
    elif path == "straight_C":
        if abs(y-border[0][1]) <= stop_d:
            stop = True
        return -(x - turning_pts_right[2][0]), d, stop, pid[0]
    elif path == "straight_D":
        if abs(x-border[1][0]) <= stop_d:
            stop = True
        return y - turning_pts_right[3][1], d, stop, pid[0]

    elif path == "right_A":
        if abs(x-border[3][0]) <= stop_d:
            stop = True
        if y < turning_pts_right[0][1]:
            return x - turning_pts_right[0][0], d, stop, pid[0]
        else:
            if x > right[0][0]:
                return -(y - right[0][1]), d, stop, pid[0]
            else:
                return -(math.sqrt((x-right_circle[0][1][0])**2 + (y-right_circle[0][1][1])**2) - right_circle[0][0]), d, stop, pid[1]
    elif path == "right_B":
        if abs(y-border[0][1]) <= stop_d:
            stop = True
        if x < turning_pts_right[1][0]:
            return -(y - turning_pts_right[1][1]), d, stop, pid[0]
        else:
            if y < right[1][1]:
                return -(x - right[1][0]), d, stop, pid[0]
            else:
                return -(math.sqrt((x-right_circle[1][1][0])**2 + (y-right_circle[1][1][1])**2) - right_circle[1][0]), d, stop, pid[1]
    elif path == "right_C":
        if abs(x-border[1][0]) <= stop_d:
            stop = True
        if y > turning_pts_right[2][1]:
            return -(x - turning_pts_right[2][0]), d, stop, pid[0]
        else:
            if x < right[2][0]:
                return (y - right[2][1]), d, stop, pid[0]
            else:
                return -(math.sqrt((x-right_circle[2][1][0])**2 + (y-right_circle[2][1][1])**2) - right_circle[2][0]), d, stop, pid[1]
    elif path == "right_D":
        if abs(y-border[2][1]) <= stop_d:
            stop = True
        if x > turning_pts_right[3][0]:
            return y - turning_pts_right[3][1], d, stop, pid[0]
        else:
            if y > right[3][1]:
                return x - right[3][0], d, stop, pid[0]
            else:
                return -(math.sqrt((x-right_circle[3][1][0])**2 + (y-right_circle[3][1][1])**2) - right_circle[3][0]), d, stop, pid[1]

    elif path == "left_A":
        if abs(x-border[1][0]) <= stop_d:
            stop = True
        if y < turning_pts_left[0][1]:
            return x - turning_pts_left[0][0], d, stop, pid[0]
        else:
            if x < left[0][0]:
                return y - left[0][1], d, stop, pid[0]
            else:
                return math.sqrt((x-left_circle[0][1][0])**2 + (y-left_circle[0][1][1])**2) - left_circle[0][0], d, stop, pid[2]
    elif path == "left_B":
        if abs(y-border[2][1]) <= stop_d:
            stop = True
        if x < turning_pts_left[1][0]:
            return -(y - turning_pts_left[1][1]), d, stop, pid[0]
        else:
            if y > left[1][1]:
                return x - left[1][0], d, stop, pid[0]
            else:
                return math.sqrt((x-left_circle[1][1][0])**2 + (y-left_circle[1][1][1])**2) - left_circle[1][0], d, stop, pid[2]
    elif path == "left_C":
        if abs(x-border[3][0]) <= stop_d:
            stop = True
        if y > turning_pts_left[2][1]:
            return -(x - turning_pts_left[2][0]), d, stop, pid[0]
        else:
            if x > left[2][0]:
                return -(y - left[2][1]), d, stop, pid[0]
            else:
                return math.sqrt((x-left_circle[2][1][0])**2 + (y-left_circle[2][1][1])**2) - left_circle[2][0], d, stop, pid[2]
    elif path == "left_D":
        if abs(y-border[0][1]) <= stop_d:
            stop = True
        if x > turning_pts_left[3][0]:
            return y - turning_pts_left[3][1], d, stop, pid[0]
        else:
            if y < left[3][1]:
                return -(x - left[3][0]), d, stop, pid[0]
            else:
                return math.sqrt((x-left_circle[3][1][0])**2 + (y-left_circle[3][1][1])**2) - left_circle[3][0], d, stop, pid[2]
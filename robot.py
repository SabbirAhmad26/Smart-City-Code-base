turning_pts_right = [(x,y)]
turning_pts_left = [(x,y)]
right = [(x,y)]
left = [(x,y)]

def project():
    if path == "straight_A":
        return x - turning_pts_right[0][0]
    elif path == "straight_B":
        return y - turning_pts_right[1][1]
    elif path == "straight_C":
        return x - turning_pts_right[2][0]
    elif path == "straight_D":
        return y - turning_pts_right[3][1]
    
    elif path == "right_A":
        if y < turning_pts_right[0][1]:
            return x - turning_pts_right[0][0]
        else:
            if x > right[0][0]:
                return y - right[0][1]
            else:
                return math.sqrt((x-right_circle[0][1][0])**2 + (y-right_circle[0][1][1])**2) - right_circle[0][0]
    elif path == "right_B":
        if x < turning_pts_right[1][0]:
            return y - turning_pts_right[1][1]
        else:
            if y < right[1][1]:
                return x - right[1][0]
            else:
                return math.sqrt((x-right_circle[1][1][0])**2 + (y-right_circle[1][1][1])**2) - right_circle[1][0]
    elif path == "right_C":
        if y > turning_pts_right[2][1]:
            return x - turning_pts_right[2][0]
        else:
            if x < right[2][0]:
                return y - right[2][1]
            else:
                return math.sqrt((x-right_circle[2][1][0])**2 + (y-right_circle[2][1][1])**2) - right_circle[2][0]
    elif path == "right_D":
        if x > turning_pts_right[3][0]:
            return y - turning_pts_right[3][1]
        else:
            if y > right[3][1]:
                return x - right[3][0]
            else:
                return math.sqrt((x-right_circle[3][1][0])**2 + (y-right_circle[3][1][1])**2) - right_circle[3][0]
            
    elif path == "left_A":
        if y < turning_pts_left[0][0]:
            return x - turning_pts_left[0][1]
        else:
            if x < left[0][0]:
                return y - left[0][1]
            else:
                return math.sqrt((x-left_circle[0][1][0])**2 + (y-left_circle[0][1][1])**2) - left_circle[0][0]
    elif path == "left_B":
        if x < turning_pts_left[1][0]:
            return y - turning_pts_left[1][1]
        else:
            if y > left[1][0]:
                return x - left[1][1]
            else:
                return math.sqrt((x-left_circle[1][1][0])**2 + (y-left_circle[1][1][1])**2) - left_circle[1][0]
    elif path == "left_C":
        if y > turning_pts_left[2][0]:
            return x - turning_pts_left[2][1]
        else:
            if x > left[2][0]:
                return y - left[2][1]
            else:
                return math.sqrt((x-left_circle[2][1][0])**2 + (y-left_circle[2][1][1])**2) - left_circle[2][0]
    elif path == "left_D":
        if x > turning_pts_left[3][0]:
            return y - turning_pts_left[3][1]
        else:
            if y < left[3][0]:
                return x - left[3][1]
            else:
                return math.sqrt((x-left_circle[3][1][0])**2 + (y-left_circle[3][1][1])**2) - left_circle[3][0]  
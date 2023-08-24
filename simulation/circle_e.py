import math

left_circle_pts = [[(451, 321), (404, 250), (331, 202)], [(332, 202), (382, 275), (450, 321)], [(452, 202), (380, 250), (331, 322)], [(331, 322), (402, 275), (453, 202)]]
right_circle_pts = [[(452, 370), (462, 341), (512, 322)], [(331, 154), (319, 184), (271, 202)], [(498, 201), (465, 188), (452, 142)], [(284, 321), (316, 332), (331, 382)]]
right_merge_e_pts = [[(451, 41), (460, 14), (491, 1)], [(738, 1), (762, 22), (770, 58)], [(770, 480), (761, 506), (722, 520)], [(492, 518), (459, 512), (451, 485)], [(770, 154), (742, 187), (703, 199)], [(737, 321), (758, 334), (772, 371)]]   #clockwise outer, top, bottom
left_merge_e_pts = [[(14, 43), (28, 13), (56, 1)], [(293, 3), (318, 13), (331, 41)], [(331, 482), (315, 508), (268, 520)], [(47, 520), (23, 492), (12, 448)], [(12, 142), (27, 185), (96, 203)], [(12, 359), (34, 332), (65, 322)]]

left_circle = []
right_circle = []
right_e = []
left_e = []

def circleRadius(b, c, d):
    temp = c[0]**2 + c[1]**2
    bc = (b[0]**2 + b[1]**2 - temp) / 2
    cd = (temp - d[0]**2 - d[1]**2) / 2
    det = (b[0] - c[0]) * (c[1] - d[1]) - (c[0] - d[0]) * (b[1] - c[1])

    if abs(det) < 1.0e-10:
        return None

    # Center of circle
    cx = (bc*(c[1] - d[1]) - cd*(b[1] - c[1])) / det
    cy = ((b[0] - c[0]) * cd - (c[0] - d[0]) * bc) / det

    radius = ((cx - b[0])**2 + (cy - b[1])**2)**.5

    return radius, (cx, cy)

def circle_lst():
    for i in left_circle_pts:
        r, c = circleRadius(i[0], i[1], i[2])
        left_circle.append([r,c])

    for i in right_circle_pts:
        r, c = circleRadius(i[0], i[1], i[2])
        right_circle.append([r,c])
    
    for i in left_merge_e_pts:
        r, c = circleRadius(i[0], i[1], i[2])
        left_e.append([r,c])

    for i in right_merge_e_pts:
        r, c = circleRadius(i[0], i[1], i[2])
        right_e.append([r,c])
  
    return left_circle, right_circle, left_e, right_e

def arc_length(circle, a, b):
    r = circle[0]
    d = math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    theta = math.acos(1-(d**2)/(2*(r**2)))
    return r*theta
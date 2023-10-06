import math
import numpy as np

left_circle_pts = [
    [(-0.592,2.93),(-0.947,3.571),(-1.713,4.05)],
    [(-1.73,2.898), (-0.945,3.35), (-0.615,4.045)],
    [(-1.717,4.056), (-1.197,3.34), (-0.576,2.927)],
    [(-0.59,4.05), (-1.235,3.605), (-1.71,2.888)]
]
right_circle_pts = [
    # [(-0.541,2.102), (-0.305,2.654), (0.102,2.9)],     #A
    # [(-2.517,2.898), (-1.926, 2.566), (-1.708, 2.099)],     #B
    # [(-1.729, 4.812), (-2.033,4.268), (-2.556, 4.064)],     #C
    # [(0.028, 4.036), (-0.394, 4.319), (-0.606, 4.853)],     #D_1
    [(-0.358, 3.696), (-0.806, 3.872), (-0.889, 4.293)],  #rightD_short_right
    [(-0.890, 6.333), (-0.785, 6.633), (-0.414, 6.696)],  #rightD_extension1_right
    [(0.117, 6.707), (0.463, 6.589), (0.521, 6.254)],   #rightD_extension2_right
    [(0.534, 4.151), (0.368, 3.8), (0.014, 3.716)],      #enter_D
    [(-2.648, 3.685), (-2.972, 3.824), (-3.042, 4.194)], #straightD_right
    [(-3.042, 4.95), (-3.017, 5.177), (-2.913, 5.368)],   #straightD_extension1_right
    [(-2.145, 6.428), (-1.88, 6.633), (-1.765, 6.686)],    #straightD_extension2_right
    [(0.117, 6.707), (0.463, 6.589), (0.521, 6.254)],   #straightD_extension3_right
    [(0.534, 4.151), (0.368, 3.8), (0.014, 3.716)]      #enter D
]

left_circle = []
right_circle = []

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

  return left_circle, right_circle

def arc_length(circle, a, b):
  # print("a:")
  # print(a)
  # a = nearest_circle_pnt(a, circle)
  # print("a corrected:")
  # print(a)
  r = circle[0]
  d = math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
  # a = (np.array(a) / d) * r
  # d = ra
  # print("d:")
  # print(d)
  # print("r:")
  # print(r)
  # print("\nb:")
  # print(b)
  # print(1-(d**2)/(2*(r**2)))
  theta = math.acos(1-(d**2)/(2*(r**2)))
  return r*theta

# def nearest_circle_pnt(pnt, circle):
#     center = circle[1]
#     r = circle[0]
#     theta = math.atan2(pnt[1]-center[1], pnt[0] - center[0])
#     pos = [r * math.cos(theta), r * math.sin(theta)]
#     return pos





# import math
# import numpy as np
#
# left_circle_pts = [
#     [(-0.592,2.93),(-0.947,3.571),(-1.713,4.05)],
#     [(-1.73,2.898), (-0.945,3.35), (-0.615,4.045)],
#     [(-1.717,4.056), (-1.197,3.34), (-0.576,2.927)],
#     [(-0.59,4.05), (-1.235,3.605), (-1.71,2.888)]
# ]
# right_circle_pts = [
#     # [(-0.541,2.102), (-0.305,2.654), (0.102,2.9)],     #A
#     # [(-2.517,2.898), (-1.926, 2.566), (-1.708, 2.099)],     #B
#     # [(-1.729, 4.812), (-2.033,4.268), (-2.556, 4.064)],     #C
#     # [(0.028, 4.036), (-0.394, 4.319), (-0.606, 4.853)],     #D_1
#     [(-0.358, 3.696), (-0.806, 3.872), (-0.889, 4.293)],  #rightD_short_right
#     [(-0.890, 6.333), (-0.785, 6.633), (-0.414, 6.696)],  #rightD_extension1_right
#     [(0.117, 6.707), (0.463, 6.589), (0.521, 6.254)],   #rightD_extension2_right
#     [(0.534, 4.151), (0.368, 3.8), (0.014, 3.716)],      #enter_D
#     [(-2.648, 3.685), (-2.972, 3.824), (-3.042, 4.194)], #straightD_right
#     [(-3.042, 4.95), (-3.017, 5.177), (-2.913, 5.368)],   #straightD_extension1_right
#     [(-2.145, 6.428), (-1.88, 6.633), (-1.541, 6.656)],    #straightD_extension2_right
#     [(0.117, 6.707), (0.463, 6.589), (0.521, 6.254)],   #straightD_extension3_right
#     [(0.534, 4.151), (0.368, 3.8), (0.014, 3.716)]      #enter D
# ]
#
# left_circle = []
# right_circle = []
#
# def circleRadius(b, c, d):
#   temp = c[0]**2 + c[1]**2
#   bc = (b[0]**2 + b[1]**2 - temp) / 2
#   cd = (temp - d[0]**2 - d[1]**2) / 2
#   det = (b[0] - c[0]) * (c[1] - d[1]) - (c[0] - d[0]) * (b[1] - c[1])
#
#   if abs(det) < 1.0e-10:
#     return None
#
#   # Center of circle
#   cx = (bc*(c[1] - d[1]) - cd*(b[1] - c[1])) / det
#   cy = ((b[0] - c[0]) * cd - (c[0] - d[0]) * bc) / det
#
#   radius = ((cx - b[0])**2 + (cy - b[1])**2)**.5
#
#   return radius, (cx, cy)
#
# def circle_lst():
#   for i in left_circle_pts:
#     r, c = circleRadius(i[0], i[1], i[2])
#     left_circle.append([r,c])
#
#   for i in right_circle_pts:
#     r, c = circleRadius(i[0], i[1], i[2])
#     right_circle.append([r,c])
#
#   return left_circle, right_circle
#
# def arc_length(circle, a, b):
#   # print("a:")
#   # print(a)
#   # a = nearest_circle_pnt(a, circle)
#   # print("a corrected:")
#   # print(a)
#   r = circle[0]
#   d = math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
#   # a = (np.array(a) / d) * r
#   # d = ra
#   # print("d:")
#   # print(d)
#   # print("r:")
#   # print(r)
#   # print("\nb:")
#   # print(b)
#   # print(1-(d**2)/(2*(r**2)))
#   theta = math.acos(1-(d**2)/(2*(r**2)))
#   return r*theta
#
# # def nearest_circle_pnt(pnt, circle):
# #     center = circle[1]
# #     r = circle[0]
# #     theta = math.atan2(pnt[1]-center[1], pnt[0] - center[0])
# #     pos = [r * math.cos(theta), r * math.sin(theta)]
# #     return pos

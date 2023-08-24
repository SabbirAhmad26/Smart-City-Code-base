import math
import numpy as np

left_circle_pts = [
    [(-0.592,2.93),(-0.947,3.571),(-1.713,4.05)],
    [(-1.73,2.898), (-0.945,3.35), (-0.615,4.045)],
    [(-1.717,4.056), (-1.197,3.34), (-0.576,2.927)],
    [(-0.59,4.05), (-1.235,3.605), (-1.71,2.888)]
]
right_circle_pts = [
    [(-0.574,2.354), (-0.45,2.758), (-0.179,2.929)],
    [(-2.275,2.875), (-1.91,2.765), (-1.716,2.51)],
    [(-1.734,4.43), (-1.838,4.178), (-2.14,4.054)],
    [(-0.273,4.054), (-0.489,4.15), (-0.604,4.44)]
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
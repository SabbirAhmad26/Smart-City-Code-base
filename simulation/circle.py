import math

left_circle_pts = [[(289, 288), (238, 220), (168, 169)], [(167, 168), (216, 239), (289, 288)], [(288, 167), (218, 215), (168, 288)], [(168, 289), (236, 238), (289, 169)]]
right_circle_pts = [[(287, 328), (296, 302), (328, 289)], [(167, 113), (156, 149), (128, 168)], [(329, 168), (298, 155), (287, 127)], [(126, 287), (155, 300), (168, 329)]]

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
  r = circle[0]
  d = math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
  theta = math.acos(1-(d**2)/(2*(r**2)))
  return r*theta
  
  
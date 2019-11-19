
from sympy import *
from sympy.geometry import *
import numpy as np

right_curve = [(2.265738794387984, 3.832994974755362), (2.2642896582850507, 3.8300747516735356), (2.2574389749568073, 3.8173856050696293), (2.3657511945452425, 3.8043457202594357), (2.813832583176618, 3.291948676560431)]

left_curve = [(2.7939942325278353, 3.1340005365606673), (2.795023440672168, 3.1318119492715404), (2.792108830990229, 3.131829976424919), (2.7749031570277283, 3.067611559813074), (3.103180766572138, 2.4518179192977714)]

left_positions = [(1.7118791495274663, 3.6699728308047512), (1.713892503741836, 3.6712180610939216), (1.7126148299619124, 3.669797166815309), (1.6827454668750599, 3.611312198027353), (1.8058882344964007, 2.9935232991143903)]
left_steering_angles = [237, 222, 225, 222, 234]

right_positions = [(1.9308408760252744, 2.9079990167840144), (1.9311937392023237, 2.90907901179699), (1.934110296233949, 2.908922355112947), (2.015967838154689, 2.8596203943866927), (2.197406132088494, 2.2096267517489245)]
right_steering_angles = [219, 400, 433, 445, 426]


# right_curve_points = []
#
# for points in right_curve:
#     x , y = points
#     right_curve_points.append(Point(x,y))
#
# first_point = True
# last_point = None
#
# right_curve_lines = []
# for points in right_curve_points:
#
#     if first_point:
#         first_point = False
#         last_point = points
#         continue
#     right_curve_lines.append(Line(points, last_point))
#     last_point = points
#
# print(right_curve_lines)

def define_circle(p1, p2, p3):
    """
    Returns the center and radius of the circle passing the given 3 points.
    In case the 3 points form a line, returns (None, infinity).
    taken from: https://stackoverflow.com/questions/28910718/give-3-points-and-a-plot-circle
    """
    temp = p2[0] * p2[0] + p2[1] * p2[1]
    bc = (p1[0] * p1[0] + p1[1] * p1[1] - temp) / 2
    cd = (temp - p3[0] * p3[0] - p3[1] * p3[1]) / 2
    det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])

    if abs(det) < 1.0e-6:
        return (None, np.inf)

    # Center of circle
    cx = (bc*(p2[1] - p3[1]) - cd*(p1[1] - p2[1])) / det
    cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det

    radius = np.sqrt((cx - p1[0])**2 + (cy - p1[1])**2)
    return ((cx, cy), radius)

right_circle = define_circle(right_positions[2], right_positions[3],right_positions[4])

left_circle = define_circle(left_positions[2], left_positions[3], left_positions[4])

print("right_circle: {}".format(right_circle))

print("left_circle: {}".format(left_circle))



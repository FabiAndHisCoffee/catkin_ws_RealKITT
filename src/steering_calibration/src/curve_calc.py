import numpy as np
import math
import statistics

right_curve = [(2.265738794387984, 3.832994974755362), (2.2642896582850507, 3.8300747516735356), (2.2574389749568073, 3.8173856050696293), (2.3657511945452425, 3.8043457202594357), (2.813832583176618, 3.291948676560431)]

left_curve = [(2.7939942325278353, 3.1340005365606673), (2.795023440672168, 3.1318119492715404), (2.792108830990229, 3.131829976424919), (2.7749031570277283, 3.067611559813074), (3.103180766572138, 2.4518179192977714)]

left_positions = [(1.7118791495274663, 3.6699728308047512), (1.713892503741836, 3.6712180610939216), (1.7126148299619124, 3.669797166815309), (1.6827454668750599, 3.611312198027353), (1.8058882344964007, 2.9935232991143903)]
left_steering_angles = [237, 222, 225, 222, 234]

right_positions = [(1.9308408760252744, 2.9079990167840144), (1.9311937392023237, 2.90907901179699), (1.934110296233949, 2.908922355112947), (2.015967838154689, 2.8596203943866927), (2.197406132088494, 2.2096267517489245)]
right_steering_angles = [219, 400, 433, 445, 426]

forward_positions = [(0.42378192432064915, 0.33506002070607566), (0.42435279829314176, 0.3354691496718358), (0.4236462438840138, 0.33504667441848135), (0.5728646731763218, 0.44697029265143173), (1.3070183416568195, 0.8514122513887797)]
forward_steering_angles = [328, 329, 328, 331, 332]


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

def calc_degrees(distance_wheels, radius):
    angle_rad = math.atan(distance_wheels / radius)
    angle = angle_rad * 180 / math.pi
    return angle

distance_wheels = 0.2

left_circle = define_circle(left_positions[2], left_positions[3], left_positions[4])
((c_left_x, c_left_y), left_radius) = left_circle
print("left_circle: {}".format(left_circle))
print("left_radius: {}".format(left_radius))
left_angle = calc_degrees(distance_wheels, left_radius)
print("left_angle: {}".format(left_angle))
left_steering_angles_clipped = left_steering_angles[1:]
print("left_steering_angles_clipped: {}".format(left_steering_angles_clipped))
left_steering_mean = statistics.mean(left_steering_angles_clipped)
print("left_steering_mean: {}".format(left_steering_mean))

right_circle = define_circle(right_positions[2], right_positions[3],right_positions[4])
((c_right_x, c_right_y), right_radius) = right_circle
print("right_circle: {}".format(right_circle))
print("right_radius: {}".format(right_radius))
right_angle = calc_degrees(distance_wheels, right_radius)
print("right_angle: {}".format(right_angle))
right_steering_angles_clipped = right_steering_angles[1:]
print("right_steering_angles_clipped: {}".format(right_steering_angles_clipped))
right_steering_mean = statistics.mean(right_steering_angles_clipped)
print("right_steering_mean: {}".format(right_steering_mean))

forward_circle = define_circle(forward_positions[2], forward_positions[3], forward_positions[4])
((c_forward_x, c_forward_y), forward_radius) = forward_circle
print("forward_circle: {}".format(forward_circle))
forward_angle = calc_degrees(distance_wheels, forward_radius)
print("forward_angle: {}".format(forward_angle))
forward_steering_angles_clipped = forward_steering_angles[1:]
print("forward_steering_angles_clipped: {}".format(forward_steering_angles_clipped))
forward_steering_mean = statistics.mean(forward_steering_angles_clipped)
print("forward_steering_mean: {}".format(forward_steering_mean))

total_angle_degree = left_angle + right_angle
print("total_angle_degree: {}".format(total_angle_degree))
total_angle_power = right_steering_mean - left_steering_mean
print("total_angle_power: {}".format(total_angle_power))

angle_per_power = total_angle_degree / total_angle_power
print("angle_per_power: {}".format(angle_per_power))

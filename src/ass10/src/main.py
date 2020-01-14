import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from scipy.interpolate import CubicSpline
import math

point_clicked = Point()


def update_point(data):
    global point_clicked
    point_clicked = data.point


def mark_points(points):
    marker = Marker()
    marker.header.frame_id ="map"
    marker.type = 4
    marker.points = points
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    return marker


def mark_point(point):
    marker = Marker()
    marker.header.frame_id ="map"
    marker.type = 2
    marker.pose.position = point
    marker.color.r = 1.0
    marker.color.a = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    return marker


def downsample_points(lane):
    data_down_sampled = lane[::50].copy()
    return data_down_sampled


def calc_spline(lane):
    return lane[:, 0], CubicSpline(lane[:, 0], lane[:, 1]), CubicSpline(lane[:, 0], lane[:, 2])


def points_from_spline(spline_x, spline_y, arc_range = np.arange(0,12.8,0.01)):
    coords_x = spline_x(arc_range)
    coords_y = spline_y(arc_range)
    points_np = np.vstack((coords_x, coords_y)).T
    points = []
    for point_np in points_np:
        point = Point(x=point_np[0], y=point_np[1], z=0.0)
        points.append(point)
    return points


def calculate_distance_between_two_points(point_1, point_2):
    x_1, y_1 = point_1
    x_2, y_2 = point_2
    return math.sqrt(((x_1 - x_2) ** 2) + ((y_1 - y_2) ** 2))

def search_closest_point_on_spline(point_closest_to, spline_x, spline_y, arc_range = (0.0,12.8), steps = 40, iterations = 6):

    # tests found, that iterations 6 is about the maximum resolution anyway
    # at iterations ~ 10, we get a RuntimeWarning: invalid value encountered
    # in double_scalars
    for interation in range(iterations):
        # arc init from search params
        arc_min, arc_max = arc_range
        step_size = (arc_max - arc_min) / steps
        search_range = np.arange(arc_min, arc_max, step_size)
        # list of (steps) points from search range
        points = np.vstack((spline_x(search_range), spline_y(search_range))).T
        # search memory init
        closest_list_number = 0
        clostest_point = points[closest_list_number]
        closest_distance = calculate_distance_between_two_points(point_closest_to, points[closest_list_number])
        # search loop through points
        for list_number, point in enumerate(points):
            distance = calculate_distance_between_two_points(point_closest_to, point)
            if distance < closest_distance:
                clostest_point = point
                closest_distance = distance
                closest_list_number = list_number
        #found closest point, also the list number of the clostest point
        # new definition of search params
        arc_min = search_range[closest_list_number] - step_size
        arc_max = search_range[closest_list_number] + step_size
        arc_range = (arc_min, arc_max)

    return clostest_point, search_range[closest_list_number]


def find_lookahead(point_closest_to, spline_x, spline_y, arc_range = (0.0,12.8), lookahead_distance = 0.5, steps = 40, iterations = 6):
    clostest_point, clostest_point_arc_distance = search_closest_point_on_spline(point_closest_to, spline_x, spline_y, arc_range = arc_range)
    lookahead_arc = clostest_point_arc_distance + lookahead_distance
    return (spline_x(lookahead_arc), spline_y(lookahead_arc))




def main():
    global point_clicked
    rospy.init_node("splines__stuff")

    sub_click = rospy.Subscriber("/clicked_point", PointStamped, update_point)
    pub_lane_marker = rospy.Publisher("/SplineMarker", Marker, queue_size=100)
    pub_clicked_point = rospy.Publisher("/ClickedPoint", Marker, queue_size=100)
    pub_closest_point = rospy.Publisher("/ClosestPoint", Marker, queue_size=100)
    pub_lookahead_point = rospy.Publisher("/LookaheadPoint", Marker, queue_size=100)

    lane1 = np.load("lane1.npy")
    lane2 = np.load("lane2.npy")

    lane1_downsampled = downsample_points(lane1)
    lane2_downsampled = downsample_points(lane2)

    lane1_arc, lane1_spline_x, lane1_spline_y = calc_spline(lane1_downsampled)
    lane2_arc, lane2_spline_x, lane2_spline_y = calc_spline(lane2_downsampled)

    lane1_points = points_from_spline(lane1_spline_x, lane1_spline_y)
    lane1_marker = mark_points(lane1_points)

    while not rospy.is_shutdown():
        pub_lane_marker.publish(lane1_marker)
        point_closest, clostest_point_arc_distance  = search_closest_point_on_spline((point_clicked.x, point_clicked.y), lane1_spline_x, lane1_spline_y)
        point_lookhead = find_lookahead((point_clicked.x, point_clicked.y), lane1_spline_x, lane1_spline_y)

        pub_clicked_point.publish(mark_point(Point(x=point_clicked.x, y=point_clicked.y, z=0.0)))
        pub_closest_point.publish(mark_point(Point(x=point_closest[0], y=point_closest[1], z=0.0)))
        pub_lookahead_point.publish(mark_point(Point(x=point_lookhead[0], y=point_lookhead[1], z=0.0)))


if __name__ == '__main__':
    main()


rospy.spin()
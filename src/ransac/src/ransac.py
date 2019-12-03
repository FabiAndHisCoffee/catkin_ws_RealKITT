import sys

import math
import random

import cv2
import numpy as np
import matplotlib.pyplot as plt

# threshold function
def lane_thresh(frame, minimum = 240, maximum = 255):
    # /sensors/camera/infra1/image_rect_raw
    ret, frame_thresh = cv2.threshold(frame, minimum, maximum, cv2.THRESH_BINARY)
    return frame_thresh

# optional roi function, not used in assignment
def roi(frame, low_right_point, top_left_point = (0,0)):
    return frame[ top_left_point[0]:low_right_point[0] , top_left_point[1]:low_right_point[1] ]

# bundling of the preprocessing steps   
def pre_processing(frame):
    frame_thresh = lane_thresh(frame)
    frame_roi = roi(frame_thresh, low_right_point = (frame_thresh.shape[0],frame_thresh.shape[1]), top_left_point = (0,0))
    return frame_roi

# the application of the line formula
# hesse normal form was tried but discarded
class LineModel():
    def __init__(self, point_a, point_b):
        # slope and anti slope
        self.gradient = (point_a[1]-point_b[1]) / (point_a[0]-point_b[0])
        self.gradient_anti = -self.gradient
        
        self.c = ( (-self.gradient_anti) * point_a[0] ) - point_a[1]
        self.offset = -self.c
        
        self.denominator = math.sqrt(pow(self.gradient_anti,2)+1)
    
    def distance_to(self, point):
        return abs((self.gradient_anti * point[0]) + point[1] + self.c) / self.denominator

# the implementation of ransac
def ransac(image_coords, threshold_percent, threshold_distance, iterations=200):  
    
    best_model = None
    best_percentage = 0
    best_candidates = None
    best_candidates_complement = None
    
    # from yx to xy... I think
    image_white_pixel_coords_inverted = np.swapaxes(image_white_pixel_coords,0,1)

    for times in range(0,iterations):
#         print("times {}".format(times))
        # find two destinct points on the image_coords
        two_random_xy_pairs = [0,0]
        while two_random_xy_pairs[0] == two_random_xy_pairs[1]:
            two_random_xy_pairs = np.random.randint(0, len(image_coords),2)
        
        # get the image coordinates of the two selected sample points
        point_a, point_b = image_coords[two_random_xy_pairs[0]], image_coords[two_random_xy_pairs[1]]
        
        # build a model with the points
        current_line_model = LineModel(point_a, point_b)
        # define a method to apply along the image_coords to get points close to the line
        def is_near_enough(point):
            return current_line_model.distance_to(point) < threshold_distance
        
        # this makes it comfortably fast, no iterating
        image_coords_bool_index = np.apply_along_axis(func1d=is_near_enough, axis=1, arr=image_coords)
        candidates = image_coords[image_coords_bool_index]

        # lets see how good this candidate is
        percentage_candidate = len(candidates)/len(image_coords)
        
#         print("candidates {} percentage_candidate {} current_line_model {}".format(len(candidates), percentage_candidate, current_line_model))
        # check candidate goodness
        if(percentage_candidate > threshold_percent and percentage_candidate > best_percentage ):  
            # yep, it's good
            best_model = current_line_model
            best_percentage = percentage_candidate
            best_candidates = candidates
            best_candidates_complement = image_coords[np.logical_not(image_coords_bool_index)]
    
    return best_model, best_candidates, best_candidates_complement

# find_lines does exactly as its name suggests. it initializes the parameters, applies ransac to the image, saves the image in a triple-array
# calc_interative_threshold is an attempt to estimate successive percentage threshold according to the amount of found pixels
def find_lines(image_coords):
    
    def calc_interative_threshold(taken, before, thresh):
        percentage_taken = taken/before
        return thresh + percentage_taken * 0.5
    
    lines = []
    iterative_thresh = 0.20
    thresh_distance = 10
    
    model, candidates, candidates_complement = ransac(image_white_pixel_coords, threshold_percent=0.20, threshold_distance = thresh_distance, iterations = 10)
    print("model {} candidates {} candidates_complement {} image_coords {}".format(model, len(candidates), len(candidates_complement),len(image_coords)))
    save_model, save_candidates, save_candidates_complement =  model, np.copy(candidates), np.copy(candidates_complement)
    lines.append((save_model, save_candidates, save_candidates_complement))
    printWhiteDots(candidates,processed.shape)
    
    iterative_thresh = calc_interative_threshold(len(candidates), len(image_coords), iterative_thresh)
    
    print("iterative_thresh {} percentage {}".format(iterative_thresh, len(candidates)/len(image_coords)))
    # how much was taken out of the start set, in percent
    
    while len(candidates_complement) > 100:
        save_length = len(candidates_complement)
        model, candidates, candidates_complement = ransac(candidates_complement, threshold_percent=iterative_thresh, threshold_distance = thresh_distance, iterations = 10)
        save_model, save_candidates, save_candidates_complement =  model, np.copy(candidates), np.copy(candidates_complement)
        lines.append((save_model, save_candidates, save_candidates_complement))
        print("iterative_thresh {} percentage {}".format(iterative_thresh, len(candidates)/len(image_coords)))
        iterative_thresh = calc_interative_threshold(len(candidates), len(image_coords), iterative_thresh)
        printWhiteDots(candidates,processed.shape)
        if candidates_complement == None:
            break
        iterative_thresh = calc_interative_threshold(len(candidates), save_length, iterative_thresh)
        
    return lines


# this method takes the parameters of a line and an image and calculates the "leftmost" and the "rightmost" intersection with the image boarders
# this is used to get the two points used to draw the line with cv2.line
def last_line_intersection(offset, gradient, max_y, max_x):
    print("max_x {} max_y {}".format(max_x, max_y))
    if offset >= 0:
        assert gradient < 0
        #two possible intersections: top or left
        x = 0
        y = offset
        while y>max_y:
            x += 1
            y += gradient
        first_point = (math.floor(x),math.floor(y))
        x, y = first_point
        while x<max_x and y>0:
            x += 1
            y += gradient
        second_point = (math.floor(x),math.floor(y+1))
        print("offset >= 0: fp {} sp {}".format(first_point, second_point))
    else: # offset < 0
        x = 0
        y = offset
        while y<0:
            x += 1
            y += gradient
        first_point = (math.floor(x),math.floor(y))
        x, y = first_point
        while x<max_x and y<max_y:
            x += 1
            y += gradient
        second_point = (math.floor(x),math.floor(y-1))
        print("offset < 0: fp {} sp {}".format(first_point, second_point))
    return first_point, second_point


def draw_lines_into_image(lines, image_grey):
    image_rgb = cv2.cvtColor(grey_image, cv2.COLOR_BGR2RGB)

    def component():
          return random.randint(0,255)

    colors = []
    for times, line in enumerate(lines):
        colors.append((component(),component(),component()))
    print(colors)
    for times, line in enumerate(lines):
        model, candidates, candidates_complement = line
        fp, lp = last_line_intersection(model.offset, model.gradient, image_rgb.shape[0], image_rgb.shape[1])
        cv2.line(image_rgb, fp, lp, colors[times], thickness=3)
    return image_rgb

import cv2 # OpenCV for perspective transform
import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import scipy.misc # For saving images as needed
import glob  # For reading in a list of images from a folder
import pandas as pd
from perception import *

# Change the path below to your data directory
# If you are in a locale (e.g., Europe) that uses ',' as the decimal separator
# change the '.' to ','
df = pd.read_csv('../test_dataset/robot_log.csv', delimiter=';', decimal='.')
csv_img_list = df["Path"].tolist() # Create list of image pathnames
# Read in ground truth map and create a 3-channel image with it
ground_truth = mpimg.imread('../calibration_images/map_bw.png')
ground_truth_3d = np.dstack((ground_truth*0, ground_truth*255, ground_truth*0)).astype(np.float)

# Creating a class to be the data container
# Will read in saved data from csv file and populate this object
# Worldmap is instantiated as 200 x 200 grids corresponding 
# to a 200m x 200m space (same size as the ground truth map: 200 x 200 pixels)
# This encompasses the full range of output position values in x and y from the sim
class Databucket():
    def __init__(self):
        self.images = csv_img_list  
        self.xpos = df["X_Position"].values
        self.ypos = df["Y_Position"].values
        self.yaw = df["Yaw"].values
        self.count = 0 # This will be a running index
        self.worldmap = np.zeros((200, 200, 3)).astype(np.float)
        self.ground_truth = ground_truth_3d # Ground truth worldmap
        self.rocks = []

class Rock():
    def __init__(self, (x, y), area):
        self.pos = [(x,y)]
        self.area = [area]  # confidence is proportional to area

    def __str__(self):
        x, y = self.position()
        return 'Rock at ({}, {}) with size {}'.format(x, y, np.mean(self.area))

    def sort(self):
        new_pos = []
        new_area = []
        for a, p in sorted(zip(self.area, self.pos)):
            new_pos.append(p)
            new_area.append(a)

        self.area = new_area
        self.pos = new_pos

    def position(self):
        # weighged position infomation
        A = sum(self.area)
        xp = 0.
        yp = 0.
        for i in xrange(len(self.area)):
            xp += self.pos[i][0] * self.area[i] / A
            yp += self.pos[i][1] * self.area[i] / A
        return int(xp), int(yp)

    def proximity_score(self, (x, y)):
        xp, yp = self.position()
        return abs(xp-x) + abs(yp-y)

    def label(self, (x, y), area):
        self.pos.append((x, y))
        self.area.append(area)
        self.sort()
        if len(self.pos) > 5:
            self.pos.pop(0)
            self.area.pop(0)

def process_image(img):
    global source, destination, data
    output_image = np.zeros((img.shape[0] + data.worldmap.shape[0], img.shape[1]*2, 3))
        
    # UPPER LEFT CORNER: original image
    output_image[0:img.shape[0], 0:img.shape[1]] = img

    # UPPDER RIGHT CORNER: warped image
    warped = perspect_transform(img, source, destination)
    output_image[0:img.shape[0], img.shape[1]:] = warped

    # BOTTOM RIGHT CORNER: navigable and obstacle threshold
    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    rock = detect_rock(bgr)
    navigable = detect_path(bgr)
    obstacle = cv2.bitwise_not(navigable)

    navigable_warped = perspect_transform(navigable, source, destination)
    obstacle_warped = perspect_transform(obstacle, source, destination)
    rock_warped = perspect_transform(rock, source, destination)

    filtered = np.zeros((rock_warped.shape[0], rock_warped.shape[1],3),np.uint8)
    xpix, ypix = np.nonzero(obstacle_warped)
    filtered[xpix, ypix, :] = [255,0,0]
    xpix, ypix = np.nonzero(navigable_warped)
    filtered[xpix, ypix, :] = [0,0,255]
    xpix, ypix = np.nonzero(rock_warped)
    filtered[xpix, ypix, :] = [255,255,0]
    output_image[img.shape[0]:img.shape[0]+navigable_warped.shape[0], img.shape[1]:,:] = filtered

    # Construct navigable map
    nav_xpix, nav_ypix = rover_coords(navigable_warped)
    if len(nav_xpix) > 0:
        nav_x_world, nav_y_world = pix_to_world(nav_xpix, nav_ypix, data.xpos[data.count], 
                data.ypos[data.count], data.yaw[data.count], 200, scale=20)
        data.worldmap[nav_y_world, nav_x_world, 2] = np.clip(data.worldmap[nav_y_world, nav_x_world,2]+15, 0, 255)
        data.worldmap[nav_y_world, nav_x_world, 0] = np.clip(data.worldmap[nav_y_world, nav_x_world,0]-15, 0, 255)
        data.ground_truth[nav_y_world, nav_x_world, 1] = np.clip(data.ground_truth[nav_y_world, nav_x_world, 1]-5,0,180)
    
    obs_xpix, obs_ypix = rover_coords(obstacle_warped)
    if len(obs_xpix) > 0:
        obs_x_world, obs_y_world = pix_to_world(obs_xpix, obs_ypix, data.xpos[data.count], 
                data.ypos[data.count], data.yaw[data.count], 200, scale=20)
        data.worldmap[obs_y_world, obs_x_world, 2] = np.clip(data.worldmap[obs_y_world, obs_x_world,2]-3, 0, 255)
        data.worldmap[obs_y_world, obs_x_world, 0] = np.clip(data.worldmap[obs_y_world, obs_x_world,0]+3, 0, 255)
        data.ground_truth[obs_y_world, obs_x_world, 1] = np.clip(data.ground_truth[obs_y_world, obs_x_world, 1]-1,0,180)
    
    rock_xpix, rock_ypix = rover_coords(rock_warped)
    if len(rock_xpix) > 5:
        rock_x_world, rock_y_world = pix_to_world(rock_xpix, rock_ypix, data.xpos[data.count], 
                data.ypos[data.count], data.yaw[data.count], 200, scale=20)
        rock_x_center = np.mean(rock_x_world)
        rock_y_center = np.mean(rock_y_world)
        rock_area = len(rock_x_world)
        matched = False
        for r in data.rocks:
            if r.proximity_score((rock_x_center, rock_y_center)) < 15:
                r.label((rock_x_center, rock_y_center), rock_area)
                matched = True
        if not matched:
            data.rocks.append(Rock((rock_x_center, rock_y_center), rock_area))
        
    # BOTTOM LEFT CORNER: world map
    # NOTE: x_world & y_world are flipped
    map_add = cv2.addWeighted(data.worldmap, 1, data.ground_truth, 0.5, 0)
    for r in data.rocks:
        x, y = r.position()
        cv2.circle(img = map_add,center = (x, y), radius = 2, color = [255,255,255], thickness = -1)
    output_image[img.shape[0]:, 0:data.worldmap.shape[1], :] = np.flipud(map_add)
        # Then putting some text over the image
    cv2.putText(output_image,"Vision-Based Mapping and Navigation", (20, 20), 
                cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
    if data.count < len(data.images) - 1:
        data.count += 1 # Keep track of the index in the Databucket()
    
    return output_image
if __name__ == '__main__':
    from moviepy.editor import VideoFileClip
    from moviepy.editor import ImageSequenceClip

    source, destination = load_trans_config('params/trans_config.txt')
    # Define pathname to save the output video
    output = '../output/mapping.mp4'
    data = Databucket()
    clip = ImageSequenceClip(data.images, fps=60) 
    new_clip = clip.fl_image(process_image) #NOTE: this function expects color images!!
    new_clip.write_videofile(output, audio=False)
    print 'Gathered {} rocks'.format(len(data.rocks))
    for r in data.rocks:
        print r

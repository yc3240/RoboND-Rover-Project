import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    '''
    transform to rover-centered view
    '''
    x_pos, y_pos = binary_img.nonzero()
    y_pixel = (binary_img.shape[1]/2 - y_pos).astype(float) #move center to zero
    x_pixel = (binary_img.shape[0] - x_pos).astype(float)   #flip by x direction
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix,ypix,yaw):
    '''
    kinematic transformation:
    x' = x * cos(yaw) - y * sin(yaw)
    y' = x * sin(yaw) + x * cos(yaw)
    '''
    # Convert yaw angle to degree
    yaw_rad = yaw * np.pi / 180.
    x_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    y_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    return x_rotated, y_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    '''
    translation and scale to global map scale
    '''
    x_world = np.int_(xpos + (xpix_rot / scale))
    y_world = np.int_(ypos + (ypix_rot / scale))
    return x_world, y_world

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    '''
    Takes in pixel level map and transform it to world sacle map
    '''
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    x_world = np.clip(np.int_(xpix_tran), 0, world_size-1)
    y_world = np.clip(np.int_(ypix_tran), 0, world_size-1)

    return x_world, y_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    source, destination = load_trans_config()
    bgr = cv2.cvtColor(Rover.img, cv2.COLOR_RGB2BGR)
    rock = detect_rock(bgr)
    navigable = detect_path(bgr)
    obstacle = cv2.bitwise_not(navigable)

    navigable_warped = perspect_transform(navigable, source, destination)
    obstacle_warped = perspect_transform(obstacle, source, destination)
    rock_warped = perspect_transform(rock, source, destination)
    
    xpix, ypix = np.nonzero(obstacle_warped)
    #Rover.vision_image[xpix, ypix, :] = [0,0,255]
    
    xpix, ypix = np.nonzero(navigable_warped)
    Rover.vision_image[:,:,:] = 0
    Rover.vision_image[xpix, ypix, :] = [255,0,0]

    xpix, ypix = np.nonzero(rock_warped)
    Rover.vision_image[xpix, ypix, :] = [255,255,0]

    
    obs_xpix, obs_ypix = rover_coords(obstacle_warped)
    if len(obs_xpix) > 0:
        obs_x_world, obs_y_world = pix_to_world(obs_xpix, obs_ypix, Rover.pos[0], 
                Rover.pos[1], Rover.yaw, 200, scale=20)
        #Rover.worldmap[obs_y_world, obs_x_world, 2] = np.clip(Rover.worldmap[obs_y_world, obs_x_world,2]-1, 0, 255)
        Rover.worldmap[obs_y_world, obs_x_world, 0] = np.clip(Rover.worldmap[obs_y_world, obs_x_world,0]+1, 0, 255)
        #Rover.ground_truth[obs_y_world, obs_x_world, 1] = np.clip(Rover.ground_truth[obs_y_world, obs_x_world, 1]-1,0,180)
    
    nav_xpix, nav_ypix = rover_coords(navigable_warped)
    if len(nav_xpix) > 0:
        # update Rover info.
        dists, angles = to_polar_coords( nav_xpix, nav_ypix )
        Rover.nav_dists = dists
        Rover.nav_angles = angles
        # update map
        nav_x_world, nav_y_world = pix_to_world(nav_xpix, nav_ypix, Rover.pos[0] \
                ,Rover.pos[1], Rover.yaw, 200, scale=20)
        Rover.worldmap[nav_y_world, nav_x_world, 2] = np.clip(Rover.worldmap[nav_y_world, nav_x_world,2]+1,0,255)
        #Rover.worldmap[nav_y_world, nav_x_world, 0] = np.clip(Rover.worldmap[nav_y_world, nav_x_world,0]-15,0,255)
        #Rover.ground_truth[nav_y_world, nav_x_world, 1] = np.clip(Rover.ground_truth[nav_y_world, nav_x_world, 1]-5,0,180)

    rock_xpix, rock_ypix = rover_coords(rock_warped)
    if len(rock_xpix) > 5 and not Rover.near_sample:
        # TODO delete this part
        if Rover.samples_pos is None:
            Rover.samples_pos = [[], []]

        # move toward the rock
        dists, angles = to_polar_coords(rock_xpix, rock_ypix )
        Rover.nav_dists = dists
        Rover.nav_angles = angles
        # update map
        rock_x_world, rock_y_world = pix_to_world(rock_xpix, rock_ypix, Rover.pos[0],
                Rover.pos[1], Rover.yaw, 200, scale=20)
        Rover.worldmap[rock_y_world, rock_x_world, 1] = np.clip(Rover.worldmap[rock_y_world, rock_x_world,1]+1,0,255)
        #Rover.samples_pos = [rock_x_world, rock_y_world]

        rock_x_center = np.mean(rock_x_world)
        rock_y_center = np.mean(rock_y_world)

        rock_area = len(rock_x_world)
        matched = False
        for idx, r in enumerate(Rover.rocks):
            if r.proximity_score((rock_x_center, rock_y_center)) < 15 and not r.collected:
                r.label((rock_x_center, rock_y_center), rock_area)
                position = r.position()
                Rover.samples_pos[0][idx] = position[0]
                Rover.samples_pos[1][idx] = position[1]
                Rover.target = r
                matched = True
        if not matched:
            Rover.rocks.append(Rock((rock_x_center, rock_y_center), rock_area))
            Rover.samples_pos[0].append(rock_x_center)
            Rover.samples_pos[1].append(rock_y_center)
            Rover.target = Rover.rocks[-1]
    #for r in Rover.rocks:
    #    x, y = r.position()
    #    cv2.circle(img = Rover.worldmap,center = (x, y), radius = 2, color = [255,255,255], thickness = -1)
    return Rover

# load preset parameters for perspective transform
def load_trans_config(filename = 'params/trans_config.txt'):
    with open(filename,'r') as f:
        lines = f.readlines()
    temp = []
    for i in range(4):
        y, x = lines[i].rstrip('\n').split(', ')
        temp.append([y, x])
    source = np.float32(temp)

    del temp[:]
    for i in range(4,8):
        y, x = lines[i].rstrip('\n').split(', ')
        temp.append([y, x])
    destination = np.float32(temp)
    return source, destination

# load preset parameters for image thresholding
def load_thresh(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
    lower = np.array(lines[0].rstrip('\n').split(', '), np.uint8)
    upper = np.array(lines[1].rstrip('\n').split(', '), np.uint8)

    return lower, upper

# compute centroid and area of a binary image
def bw_info(mask):
    x, y = np.nonzero(mask)
    if len(x) == 0:
        return -1,-1,0
    xc = np.mean(x, dtype = int)
    yc = np.mean(y, dtype = int)
    area = len(x)   # area is proportional to confidence
    return xc, yc, area

# threshold rock pixels
def detect_rock(img):
    rock_lower, rock_upper = load_thresh('params/rock_thresh.txt')
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, rock_lower, rock_upper)
    
    # morphological processing
    kernel = np.ones((3,3),np.int8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

# threshold navigable images
def detect_path(img):
    path_lower, path_upper = load_thresh('params/path_thresh.txt')
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    navigable = cv2.inRange(hsv, path_lower, path_upper)
    return navigable

class Rock():
    def __init__(self, (x, y), area):
        self.pos = [(x,y)]
        self.area = [area]  # confidence is proportional to area
        self.collected = False

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

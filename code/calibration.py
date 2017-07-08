import cv2
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
from perception import perspect_transform, load_trans_config

BOTTOM_OFFSET = 6
DEST_BOX_SIZE = 5

# callback function for GUI
RED = (0,0,255)
ref_pt = []
def click_and_crop(event, x, y, flags, image):
    global ref_pt
    if event == cv2.EVENT_LBUTTONUP:
        ref_pt.append((x,y))
        cv2.circle(image, ref_pt[-1], radius = 2, color = RED, thickness = -1)
        cv2.imshow('image',image)

# GUI for pick 4 transformation reference points
# press 'r' to reset
# press 'c' to leave
def pick_obj(img_path):
    global ref_pt
    image = cv2.imread(img_path)
    clone = image.copy()
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', click_and_crop, image)

    while len(ref_pt) < 4:
        cv2.imshow('image', image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('r'):
            image = clone.copy()
            cv2.setMouseCallback('image', click_and_crop, image)

        if key == ord('c'):
            break

    cv2.destroyAllWindows()

    # note that x and y coords are flipped in cv2
    reverted = []
    for y,x in ref_pt:
        reverted.append((x,y))
    h,w,c = image.shape
    return sort_points(reverted, (h,w))

# Sort four points by 
# bottom-left -> bottom-right -> upper-right -> upper-left
def sort_points(points, dimensions):
    # find left bottom corner
    min_dist = 99999
    lbc = None  #left-bottom corner
    min_idx = 0
    for i,(x, y) in enumerate(points):
        dist = abs(y) + abs(dimensions[0]-x)
        if lbc == None or dist < min_dist:
            lbc = (x, y)
            min_dist = dist
            min_idx = i
    # calculate angle relative to left-bottom corner
    angles = []
    for i,(x, y) in enumerate(points):
        if i == min_idx:
            angles.append(-99999)
        else:
            angles.append(np.arctan2(y-lbc[1], x-lbc[0]))
    
    # return points sorted by angles
    return [point for angles, point in sorted(zip(angles, points))]



if __name__ == "__main__":
    import sys
    # pick tranform points
    points = pick_obj(sys.argv[1])
    print points[0][1]
    if len(points) != 4:
        raise ValueError('Pick exactly 4 points.')
    
    source = np.float32([[points[0][1], (points[0][0]+points[1][0])/2 ], \
                         [points[1][1], (points[0][0]+points[1][0])/2 ], \
                         [points[2][1], (points[2][0]+points[3][0])/2 ], \
                         [points[3][1], (points[2][0]+points[3][0])/2 ]  \
                         ])
    image = cv2.imread(sys.argv[1])
    destination = np.float32([[image.shape[1]/2-DEST_BOX_SIZE, image.shape[0]-BOTTOM_OFFSET],
            [image.shape[1]/2+DEST_BOX_SIZE, image.shape[0]-BOTTOM_OFFSET],
            [image.shape[1]/2+DEST_BOX_SIZE, image.shape[0]-BOTTOM_OFFSET-2*DEST_BOX_SIZE],
            [image.shape[1]/2-DEST_BOX_SIZE, image.shape[0]-BOTTOM_OFFSET-2*DEST_BOX_SIZE]])
    
    warped = perspect_transform(image,source,destination)
    cv2.imshow('Warped', warped)
    cv2.waitKey(0)

    option = raw_input('Save this configuration [y/n] ?  ')
    if option == 'y' or option == 'Y':
        with open('trans_config.txt','w') as f:
            # first four lines are source
            for y,x in source:
                f.writelines(str(y)+', '+str(x)+'\n')
            for y,x in destination:
                f.writelines(str(y)+', '+str(x)+'\n')
    
    # EXAMPLE: THIS IS HOW YOU CAN GET TRANSFORMATION CONFIG. FROM TEXT FILE
    #source_load, destination_load = load_trans_config('trans_config.txt')
    #print 'Load source: '
    #print source_load
    #print 'Load destination: '
    #print destination_load

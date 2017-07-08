import sys
import cv2
import glob
import numpy as np
from perception import perspect_transform, load_trans_config \
        ,load_thresh, bw_info, detect_rock, detect_path

filepaths = glob.glob(sys.argv[1]+'/*.jpg')
for filepath in filepaths:
    image = cv2.imread(filepath)
    source, destination = load_trans_config('params/trans_config.txt')
    bw = detect_path(image)
    bw = perspect_transform(bw, source, destination)
    xc, yc, area = bw_info(detect_rock(image))
    if xc != -1:
        r = int(np.sqrt(area))
        cv2.circle(image, (yc, xc), radius=r,color=(0,0,255))
    cv2.imshow('image', image)
    cv2.imshow('path', bw)
    cv2.waitKey(0)
cv2.destroyAllWindows()

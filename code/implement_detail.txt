Udacity Robotics Nanodegree
Project 1: Autonomous Navigation
Name: Yan-Song Chen

---------------------------------CONTENT---------------------------------------
[REQUIRED]
1. README.txt
   This file explains the implementation details and writeup of this project.

2. video.py (REQUIRED)
   Sorry grader... but I prefer to program in vim. Therefore I wrote the video
   analysis in this file. Essentially these are the same program as jupyter
   notebook.

3. drive_rover.py
   The driver program. I commented the printout.

4. perception.py
   Process robocam images and detect path and rocks. It is pretty much similar
   to video.py.

5. decision.py
   Tell the robot to make decision based on perception.

6. supporting_functions.py

7. params/trans_config.txt
   Stores the "source" and "destination" matrix for perspective transformation.

8. params/path_thresh.txt
   Stores preset HSV threshold value for path detection.

9. params/rock_thresh.txt
   Stores preset HSV threshold value for rock detection.

[PERIPHERAL PROGRAM]
1. filter.py
   Shows the result of using threshol of REQUIRED 7/8/9.

2. calibration.py
   GUI for picking calibration matrix.

------------------------------VIDEO ANALYSIS-----------------------------------
[REQUIRED]
1. Before runnning video.py, make sure your directory of "RoboND-Rover-Project"
   has sub-directories "code", "output" and "test_dataset". Python scripts 
   should be placed in "code"; Directory "params" should be in "code"; 
   "test_dataset" should be the directory where IMG and robot_log.csv are 
   stored. Here is the suggested directory.

  - RoboND-Rover-Project
      - code
          - video.py
          - params
              - trans_config.txt
      - output
      - test_dataset
          - IMG
          - robot_log.txt

2. Run video.py on command line.

   $ python video.py

   The output video "mapping.mp4" sholud be in file output.

3. My result can be view on Youtube.
   Run 1: https://youtu.be/6V07AOrDj24

[PERIPHERAL PROGRAM]
1. If you were to calibrate the perspective transform, run calibration.py

   $ python filter.py CALIBRATION_IMAGE.EXTENSION

   The image window will pop out. Pick four corner points. If you want to redo
   picking, please press 'r'. For cancelling press 'c'.

   After you clicked on four points, the warped image will show up. If you
   need to store this configuration in "params", enter 'Y' on command line. 
   Otherwise the configuration will not be saved.

2. If you want to see the threshold result, run filter.py

   $ python filter.py IMAGE_FILE

   The program will show the binary image and the original image in two 
   windows for each picture in the file. Press any key to see next picture.
   If the image contains a rock sample, it will be encircled by red.

[IMPLEMENTATION DETAIL]
In process_image(), I used HSV format because it is more stable to intensity
and color variation. I arranged the robot vision images (or the thresheld 
warped image) in the bottom right section. In the robot vision, blue is 
navigable, red is obstacle and yellow is rock sample. On top of color 
thresholding, detect_rock() also applies morphological processing to reduce
noise.

The world map was constructed based on the world pixels of robot vision images.
Because the navigable region is usually less than the obstacle, I increase the
navigable by 15 each iteration. Also to eliminate ambibuity, I decrease other
color channel by the same amount.

In order to track the position of rock, I wrote the class "Rock". It will 
compare proximity of a new sample from robot vision and tell if the rock in the
vision belongs to this instance. The proximity score is based on manhattan 
distance for simplicity. An Rock instance stores 5 measurements. Each 
measurement is weighted by their area, because the condidence of each 
measurement is assumed to be proportional to their area. The position() method
calculates the weighted position among the five measurements.

Finally, the Databucket has a list of Rocks. The weighted position of each rock
will be plotted on the worldmap as white circles.

----------------------------AUTONOMOUS SIMULATION------------------------------
1. The configurations in my autonomous simulation are
  - resolution: 1024 x 768
  - image quality: good
  - fps: 23 ~ 27

2. Run autonommous mode by 

   $ python drive_rover.py

   After executing this command, open the simulator and press autonomous mode.
   The rover will be running autonomously.

[IMPLEMENTATION DETAIL]
1. drive_rover.py
   Several member variable and methods were added.

   - target: specify whether a rock is detected. If more than 5 yellowish 
             pixels are filtered out, then perception label Rover.target as 
             True. Othrewise Rover.target is False.

   - steer_dir: in stop mode, determine the direction of steering by the 
                majority of navigable pixels.

   - counter: this is for controling execution time of each state.

   - prev_pos: this is to compare whether the robot is moving and to determine
               whether to change state.
   
   - unstable(): if pitch and roll angles are more than +-15 degree, then 
                 report unstable. This comes handy to mapping because 
                 unstablility affects the accuracy of perpective transform a
                 lot.

2. perception.py
   Perception step is very similar to video.py. It thresholds the warped images
   by HSV bounds. However, I did not use Rock class in this case. More simiply,
   I used a boolean label Rover.target to specify whether a rock is detected. 

3. decision.py
   Decision step is primarily comprised of five modes.

   - forward mode: 
      * Throttle: keep accelerating as velocity is below maximum limit. 
      * Steer angle: determined by the mean of navigable angles.
      * Mode transition:
          If any target is identified by perception, switch to search mode.
          If over 500 iterations has passed and displacement is less than 2, 
          switch to backward mode.

   - stop mode:
      If number of navigable pixels is below the threshold, keep rotating to the
      direction of majarity navigable pixels. Otherwise, switch to forward mode.

   - search mode:
     * Throttle: half of forward mode.
     * Steer angle:
        If any target is specified, pick the average of 5 nearest angles as 
        steer angle. This will be useful when two rocks are in the robot vision.
        Otherwise, follow the mean of navigable angles.
     * Mode transition:
        If the target is missing or the robot is not moving over 300 iterations,
        switch to backward mode.
        If the robot is near sample, pick up the sample and switch to stop mode.

   - backward mode:
      Go backward by 80 iterations. Upon finish, transit to wait mode.

   - wait mode:
      Lock down the brake for 35 iterations and wait the robot toe stablize.
      Upon finish, transit to stop mode to restart navigation.

   Flow chart:
                             (t > 500)
           forward ---------------------------> backward
            ^   \                    (t > 300)   ^  |
            |    '-----------search--------------'  |
(navigable) |    (pick up)     |                    |(t = 80)
            | .----------------'                    |
            | v              (t = 35)               v
           stop <-------------------------------- wait

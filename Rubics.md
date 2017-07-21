# Rubics

## Notebook Analysis

### Implementation Detail
In process_image(), I used HSV format because it is more stable to intensity and color variation. I arranged the robot vision images (or the thresheld warped image) in the bottom right section. In the robot vision, blue is navigable, red is obstacle and yellow is rock sample. On top of color thresholding, detect_rock() also applies morphological processing to reduce noise.

The world map was constructed based on the world pixels of robot vision images. Because the navigable region is usually less than the obstacle, I increase the navigable by 15 each iteration. Also to eliminate ambibuity, I decrease other color channel by the same amount.

In order to track the position of rock, I wrote the class "Rock". It will compare proximity of a new sample from robot vision and tell if the rock in the vision belongs to this instance. The proximity score is based on manhattan distance for simplicity. An Rock instance stores 5 measurements. Each measurement is weighted by their area, because the condidence of each measurement is assumed to be proportional to their area. The position() method calculates the weighted position among the five measurements.

Finally, the Databucket has a list of Rocks. The weighted position of each rock will be plotted on the worldmap as white circles.

### Video

Please look at the youtube link: 
Run 1: [https://www.youtube.com/edit?o=U&video_id=6V07AOrDj24]
Run 2: [https://www.youtube.com/edit?o=U&video_id=Okv85XSa1-A]


## Autonomous Navigation and Mapping

### Implementation Detail
Perception step is very similar to video.py. It thresholds the warped images by HSV bounds. If the rock image has more than 5 pixels, it will be considered as an rock. The rock vision will then be examined by proximity score and stored in the RoverState instance.

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

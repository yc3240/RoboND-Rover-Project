import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # initialization
    if Rover.prev_pos is None:
        Rover.prev_pos = Rover.pos
        Rover.counter = 600

    # transit modes whenever counter is down
    if Rover.counter < 0:
        if Rover.mode == 'forward':
            Rover.mode = 'backward'
            Rover.counter = 80
            print 'Switch to {} mode'.format(Rover.mode)
        elif Rover.mode == 'search':
            Rover.mode = 'backward'
            Rover.counter = 80
            print 'Switch to {} mode'.format(Rover.mode)
        elif Rover.mode == 'backward':
            Rover.mode = 'wait'
            Rover.counter = 35
            print 'Switch to {} mode'.format(Rover.mode)
        elif Rover.mode == 'wait':
            Rover.mode = 'stop'
            Rover.counter = 20
            print 'Switch to {} mode'.format(Rover.mode)
            
    if Rover.unstable():
        if Rover.nav_angles is not None:
            Rover.steer_dir = np.sign(np.mean(Rover.nav_angles))
        if not Rover.near_sample and Rover.mode != 'stop':
            Rover.mode = 'stop'
            print 'Switch to {} mode'.format(Rover.mode)
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            if np.sqrt((Rover.prev_pos[1]-Rover.pos[1])**2+(Rover.prev_pos[0]-Rover.pos[0])**2) < 2:
                Rover.counter -= 1
            else:
                Rover.prev_pos = Rover.pos
                Rover.counter = 600
            # Check the extent of navigable terrain
            if Rover.target is not None:
                Rover.mode = 'search'
                Rover.counter = 300
                print 'Switch to {} mode'.format(Rover.mode)
            elif len(Rover.nav_angles) >= Rover.stop_forward: 
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.steer_dir = np.sign(np.mean(Rover.nav_angles))
                    Rover.mode = 'stop'
                    Rover.counter = 0  # stop mode does not count
                    print 'Switch to {} mode'.format(Rover.mode)
                    

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    # Turn to the direction with more navigable pixels.
                    Rover.steer = 15 * Rover.steer_dir
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
                    Rover.counter = 600
                    print 'Switch to {} mode'.format(Rover.mode)
                    
        elif Rover.mode == 'search':
            if Rover.target is None:
                # target is unidentified, use mean angle
                Rover.counter -= 1
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                Rover.throttle = 0.1
            else:
                if np.sqrt((Rover.prev_pos[1]-Rover.pos[1])**2+(Rover.prev_pos[0]-Rover.pos[0])**2) < 2:
                    Rover.counter -= 1
                else:
                    Rover.prev_pos = Rover.pos
                    Rover.counter = 300
                    #dist = np.sqrt( (target_pos[0]-Rover.pos[0])**2+(target_pos[1]-Rover.pos[1])**2)
                    #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                # target is identified, use the nearest 5 angles to prevent outlier
                peak_angles = np.array([y for x,y in sorted(zip(Rover.nav_dists, Rover.nav_angles))[0:5]])
                Rover.steer = np.clip(np.mean(peak_angles * 180/np.pi), -15, 15)
                Rover.throttle = 0.1
                Rover.brake = 0
            # stop to pickup the rock
            if Rover.near_sample:
                Rover.throttle = 0.0
                Rover.brake = Rover.brake_set

        elif Rover.mode == 'backward':
            Rover.steer = 0
            Rover.throttle = -0.4
            Rover.counter -= 1

        elif Rover.mode == 'wait':
            Rover.steer = 0
            Rover.throttle = 0
            Rover.counter -= 1
            Rover.brake = Rover.brake_set
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = 0#Rover.throttle_set
        Rover.steer = -15
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        if Rover.target is not None:
            Rover.target.collected = True
        Rover.target = None
        Rover.send_pickup = True
        Rover.mode = 'stop'
        Rover.counter = 40
        print 'Switch to {}'.format(Rover.mode)
    
    return Rover


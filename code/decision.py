import numpy as np
import time
import math


def check_for_sample2(Rover):
    if Rover.near_sample == True and Rover.mode == 'forward':
        # stop to pickup sample
        Rovere = prepare_to_stop(Rover)
        Rover.picking_sample = 1
    elif Rover.near_sample == True and Rover.mode == 'stop'  and (Rover.send_pickup_cmd == 0 or Rover.send_pickup_cmd == None):
        # send command to pickup
        Rover.send_pickup_cmd = 1
    return Rover

def prepare_to_stop(Rover):
    # Set mode to "stop" and hit the brakes!
    Rover.throttle = 0
    # Set brake to stored brake value
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    Rover.mode = 'stop'
    return Rover

# This function will detect if a sample was find, even if flag near_sample is still false
# When the sample is detected, the function will use postion of sample and rover and guide
# the rover in direction of sample. Then near_sample is enabled, the rover is stopped and
# the pickup command is sent. The function waits until near_sample is disabled, which means
# there is no sample near, or the pick up sample was success. Then all the flags that keeps
# the rover in pick up state are disabled, the break is set to zero and mode set to forward.
# This way, in the next iteration the rover will continue to execute the regular procedures.
# A PD controller was added to acellerate the pickup process.
#-----------------------------------------------------------------------------------------

def check_for_sample(Rover):
    # Check whether any rock detections are present in worldmap
    rock_world_pos = Rover.worldmap[:,:,1].nonzero()
    # if found sample near, go pick up
    if Rover.picking_sample:
        kp = .0005
        ka = .1
        kb = - .001
        alpha = 0
        beta = 0
        if Rover.near_sample == 1 and Rover.mode == 'forward':
            Rover.brake = Rover.brake_set
            print('oi')
            Rover.steer = 0
            Rover.mode = 'stop'
            return Rover
        elif Rover.near_sample == 1 and Rover.send_pickup_cmd == 0 and Rover.mode == 'stop':
            Rover.send_pickup_cmd = 1
            return Rover
        elif Rover.near_sample == 1 and Rover.send_pickup_cmd == 1 and Rover.wait_pickup == 0:
            Rover.wait_pickup = 1
            Rover.send_pickup_cmd = 0
            return Rover
        elif Rover.near_sample == 0 and Rover.wait_pickup == 1:
            Rover.wait_pickup = 0
            Rover.picking_sample = 0
            Rover.mode = 'forward'
            return Rover
        # Now use the sample as target
        idx = np.sum(Rover.samples_found)-1
        deltaX = Rover.pos[0] - Rover.samples_pos[0][idx]
        deltaY = Rover.pos[1] - Rover.samples_pos[1][idx]
        deltaXabs = np.abs(deltaX)
        deltaYabs = np.abs(deltaY)
        arctanXY = math.atan2(deltaXabs, deltaYabs)
        arctanYX = math.atan2(deltaYabs, deltaXabs)
        theta = Rover.yaw * np.pi / 180
        sign = 1
        rho = math.sqrt(deltaX**2 + deltaY**2)
        if deltaX < 0 and deltaY < 0:   # quadrant 1
            if theta > arctanYX:
                alpha = theta - arctanYX
                beta = - theta + alpha
                sign = -1
            else:
                alpha = arctanYX - theta
                beta = -theta - alpha
        elif deltaX > 0 and deltaY < 0:
            arctan = arctanXY
            if arctan + np.pi/2 < theta:
                alpha = -arctan + theta - np.pi/2
                beta = -theta + alpha
                sign = -1
            else:
                alpha = theta - arctan
                beta = -alpha - theta
        Rover.throttle = kp * rho
        w = (ka * alpha) + (kb * beta)
        #Rover.steer = (sign * w) * 180/np.pi
        Rover.steer = (sign * w)
        print('throt2', Rover.throttle, 'steer2', Rover.steer)
        print('alpha', alpha, 'beta', beta, 'w', w, 'deltaX', deltaX, 'deltaY', deltaY, 'arcXY', arctanXY, 'arcYX', arctanYX,
                'sample', Rover.samples_pos[0][idx],Rover.samples_pos[1][idx])
        return Rover
    else:
        # If there are, we'll step through the known sample positions
        # to confirm whether detections are real
        if rock_world_pos[0].any():
            rock_size = 2
            for idx in range(len(Rover.samples_pos[0]) - 1):
                test_rock_x = Rover.samples_pos[0][idx]
                test_rock_y = Rover.samples_pos[1][idx]
                rock_sample_dists = np.sqrt((test_rock_x - rock_world_pos[1])**2 + \
                                            (test_rock_y - rock_world_pos[0])**2)
                # If rocks were detected within 3 meters of known sample positions
                # consider it a success and plot the location of the known
                # sample on the map
                if np.min(rock_sample_dists) < 3:
                    Rover.picking_sample = 1
                    Rover.throttle = Rover.throttle_set
                    #check_for_sample(Rover)
                    #print(rock_sample_dists)
                            #Rover.samples_found[idx] = 1
                            #map_add[test_rock_y-rock_size:test_rock_y+rock_size, 
                            #test_rock_x-rock_size:test_rock_x+rock_size, :] = 255
    return Rover

# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    # 'odo', ("%.2f" % Rover.odometry)
    print('mode', Rover.mode, 'dist', np.mean(Rover.nav_dists), 'angle', np.mean(Rover.nav_angles), 'near', Rover.near_sample, 
    'picking', Rover.picking_sample, 'send', Rover.send_pickup_cmd)
    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check if is picking up samples
        Rover = check_for_sample(Rover)
        if Rover.picking_sample:
            return Rover    
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            vel = np.abs(Rover.vel)
            dt = time.time() - Rover.last_time
            Rover.last_time = time.time()      # save current time
            Rover.odometry += (vel * dt)        # update odometer
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < 0 or Rover.vel == 0:
                    prepare_to_turn(Rover)
                    return Rover
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
                prepare_to_stop(Rover)

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            #Rover.picking_up = 0
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
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set

    return Rover

def prepare_to_turn(Rover):
    Rover.mode = 'stop'
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
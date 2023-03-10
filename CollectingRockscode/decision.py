import numpy as np
#import perception

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
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
                    Rover.mode = 'stop'

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
                    
        elif Rover.mode=='backward':
            Rover.rotateCounter=0 
           # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                #if len(Rover.nav_angles) < Rover.go_forward:
                Rover.throttle = -0.2
		    # Release the brake to allow turning
                Rover.brake = 0
		    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                Rover.steer = 0 # Could be more clever here about which way to turn
                Rover.mode='rotate'
		# If we're stopped but see sufficient navigable terrain in front then go!
            
        elif Rover.mode=='rotate':
          # If we're in stop mode but still moving keep braking
          #3amalt rover.vel < 0 3ashan lma bygy y3mel rotation bytb2a 2a2al men el zero 
            if Rover.vel > 0.05 or Rover.vel <0:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.05:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward or Rover.rotateCounter<6:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    if(Rover.go_left):
                       Rover.steer=15
                    else:
                       Rover.steer=-15    
                    #Rover.steer = -15 # Could be more clever here about which way to turn 180 right direction
                    if(Rover.rotateCounter<6):
                       Rover.rotateCounter+=1
                       
                    Rover.mode='rotate'      
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward :
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
                    
            elif Rover.mode=='sample':
        
             ###############1) Move towards the found samples############### 
                  # check that there is sample data available
                  #el sample distance bytb2a initialized by null
                  #el sample distance hya el distance mabeen el rover wel sample
               if len(Rover.sample_dists) > 0: 
                  distance = np.mean(Rover.sample_dists)
               else:
                  distance = 30
               if len(Rover.sample_angles) > 0:
                  steer = np.mean(Rover.sample_angles * 180 / np.pi)
               else:
                 steer = 0
             #law mala2ash sample_dists hay7ot default distance
             #law mala2ash sample_angles hay7ot default steer by zero 

             ###############2) check if the sample can be picked############### 
                   # check if the sample can be picked up
               if Rover.near_sample > 0:
                 if Rover.vel > 0.2:
                   # travelling to fast so stop
                      Rover.throttle = 0
                      Rover.brake = 0.1
                      Rover.steer = np.clip(steer, -15, 15)
                 elif Rover.vel <= 0.1:
                  # the sample can be picked up so pick it up
                    Rover.mode = 'stop'
                    Rover.send_pickup = True
                    Rover.mode = 'rotate'

      
                      
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

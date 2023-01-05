import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import scipy.misc
import glob
import imageio
import decision

new_nav_angles =[]

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
#color_thresh
############purpose of this function############:
#We need to identify pixels that are above certain value of threshold 
#In case of the ground pixels we can use value of RGB threshold greater than 160 
#Apply that threshold on the input image

############Input of the function############:
#1-Input Image (the theshold will be applied to it)
#2-RGB threshold value

############output of the function############:
#Binary image that contain ones in the pixels above the threshold values.

############Implementation of the function############: 
#1-Create an aray of zeros with size X*Y of the image, the array will be single channel
#color_select = np.zeros_like(img[:,:,0])

#2-Check that each pixel of the array is above the three threshold values in RGB
#  First  condition: pixels values above the Red   threshold
#  Second condition: pixels values above the Green threshold
#  Third  condition: pixels values above the Blue  threshold
# above_thresh will now contain a boolean array with "True" where threshold was met 
    
# 3- the index of the color_select array will be accessed by above_thresh
#    Set the value of that pixel with one to produce a binary image.   
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
    
#threshed = color_thresh(warped)
#plt.imshow(threshed, cmap='gray')

 
# Define a function to convert from image coords to rover coords
# rover_coords
############With respect to rover############:
#The x-axis direction of the rover is is located at the center of the rover pointing in the forward #direction of the navigable train (with respect to the rover)


############With respect to Us############:
#the origin is at the bottom left and the x-axis positive direction is pointing towards the right #direction


############purpose of this function############:
#map the coordinates of the rover (that have different x-axis direction) into an image that have the #origin at the left and have x-axis pointing towards the right direction


############output of the function############:
#x-pixel , y-pixel are the new coordinates of the rover in the new image that have the origin at the left #and have x-axis pointing towards the right direction

############Implementation of the function############: 
#there are three operations
#1-mapping: the x-axis of the rover will be the y-axis of the new image
          #: the y-axis of the rover will be the x-axis of the new image
#2-Then  Use the equation of the Reflection

##NOTE:
# binary_img.shape[0]: represents the height the image(number of rows pixels)    
#(binary_img.shape[1]/2) represents the width of the image divided by 2(column pixels/2)
def rover_coords(binary_img):
    # Identify nonzero (white = navigable) pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
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


# Define a function to map rover space pixels to world space
#rotate_pix function
############purpose of this function############:
#Map the axis of the rover x and the rover y to the world x and world y 
#this could be done by the rotation around the z-axis with the yaw rate of the robot

############output of the function############:
#the axis of the rover x and the rover y will be parallel to the axis of the world x and world y
#the function will return the x_pixel and y_pixel after rotation
  
############Implementation of the function############: 
#1- convert the yaw angle to radian by multiplying with (pi/180) 
#2- Multiply the position vector of the pixel with rotation matrix
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    # Rotation Matrix:
    #[x' y']T = [cos theta  -sin theta,  * [x y]T
    #            sin theta  cos theta]
    # theta will be yaw angle
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

#translate_pix function
############purpose of this function############:
#Apply scaling to adjust/fix the size of objects in the world map and translation to reposition the rover x and y axis to start from the origin
#of the world's x and y axis.

############output of the function############:
#the size of objects in the map will be adjusted/scaled 
#the axis of the rover x and the rover y will be poistioned at/will originate from the origin of the world x and world y
#the function will return the x_pixel and y_pixel after translation and scaling 
  
############Implementation of the function############: 
#1- divide the input x and y values by scale value
#2- add xpos and ypos respectively to the x and y values to apply translation 
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    # Translation and DIVIDING BY SCALE
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
#pix_to_world function
############purpose of this function############:
#Call the rotate_pix function and the translate_pix function as well as apply clipping.

############output of the function############:
#the function will return the x_pixel and y_pixel values after rotation, scaling, translation, and clipping

############Implementation of the function############: 
#1- call rotate_pix and translate_pix
#2- clip the x and y values using np.clip
#3- return the final x and y values that will be mapped onto the world
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world
    

# Define a function to perform a perspective transform
#perspect_transform
############purpose of this function############:
#1-Generate the mask of the bird eye view 
#2-apply the mask on the image to produce an image with the bird eye view

############Implementation of the function############: 
#Step1:
#M = cv2.getPerspectiveTransform(src, dst) 
#generation of the mask is done by having src points and dst points in order to estimate the value of the M

#Step2:
#warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))
#apply the filter M on the image (img) and 
#the result will be a new image with dimention (img.shape[1], img.shape[0]) which is the same size of the input image
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0])) #keep same size as input image (input y then x)
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]),M,(img.shape[1],img.shape[0]))
    return warped, mask

# Color threshing, but rocks 
def find_rocks(img, levels=(110,110,50)):
    rockpix = ((img[:,:,0] > levels[0]) & (img[:,:,1] > levels[1]) & (img[:,:,2] < levels[2]))
    color_select = np.zeros_like(img[:,:,0])
    color_select[rockpix] = 1
    
    return color_select


def generate_new_nav_angles(threshed,Rover):
    #get all nav pixels xpos, ypos
    ones = np.argwhere(threshed==1)

    x = [i[0] for i in ones]
    y = [i[1] for i in ones]

    if len(x)<12:
       Rover.throttle=0
       Rover.brake = Rover.brake_set
       Rover.steer=0
       Rover.mode='backward'
       ones = np.argwhere(threshed==1)
       x = [i[0] for i in ones]
       y = [i[1] for i in ones]
       
    if len(x)>0:
       coeff = np.polyfit(x, y, deg=2) 
       yvals = np.polyval(coeff, x) #get y vals from x vals
       for i in x:
           new_nav_angles=(np.arctan2(np.polyval(coeff, i),i)) #angles for rover path
    else:
       new_nav_angles=0      
  
    
     
   
    #plt.plot(x, yvals)
    #new_nav_angles = []
        
     
        
    return  Rover   
		    

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):

	#debugger flag
    debugger = False # to enable debugging mode, set to True
   
   
    """
    We will return the updated Rover. Update two params: Rover.vision_image and Rover.worldmap
    """   
   
   # We first calculate warped Rover perspective and extract the mask
   # Perspective is warped to bird eye view
   # We use 10 by 10 pixels to be destination size for one square meter of the grid
   # Later, we will map the 10 by 10 pixels to the worldmap

#perspect_transform
############purpose of this function############:
#1-Generate the mask of the bird eye view 
#2-apply the mask on the image to produce an image with the bird eye view

############Implementation of the function############: 
#Step1:
#M = cv2.getPerspectiveTransform(src, dst) 
#generation of the mask is done by having src points and dst points in order to estimate the value of the M

#Step2:
#warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))
#apply the filter M on the image (img) and 
#the result will be a new image with dimention (img.shape[1], img.shape[0]) which is the same size of the input image

    dst_size = 3
    bottom_offset = 6
    image = Rover.img
    source = np.float32([[14, 140], # bottom left
                 [301 ,140], # bottom right
                 [200, 96], 
                 [118, 96]]) # arbitrarily clockwise or anti-clockwise square (same as destination)
    destination = np.float32([
            [image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
            [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
            [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset],
            [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
            ])
    
    warped, mask = perspect_transform(Rover.img, source, destination)
    # Color thresholding function will return B&W image, where navigable = 1
    threshed = color_thresh(warped)
    Rover = generate_new_nav_angles(threshed,Rover) 
    # Using perspect_transform mask
    # Note: obs_map not yet worldmap. Needs transformation and mapping
    obs_map = np.absolute(np.float32(threshed)-1) * mask 
    
    
    Rover.vision_image[:,:,2] = threshed * 255 #b navigable
    Rover.vision_image[:,:,0] = obs_map * 255 #r obs
    #Rover.vision_image[:,:,1]  #g rock
         


    # Mapping the warped threshed image into the world map
    # Reflection and centering
    xpix, ypix = rover_coords(threshed)
    world_size = Rover.worldmap.shape[0] # init world_size so we can map to it
    scale = 10  
    # Scaling 10 by 10 pixels in front of robot to world size
    x_world, y_world = pix_to_world(xpix, ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    
    obsxpix, obsypix = rover_coords(obs_map) 
    obs_x_world, obs_y_world = pix_to_world(obsxpix, obsypix, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    
    Rover.worldmap[y_world, x_world,2] += 10
    Rover.worldmap[obs_y_world , obs_x_world , 0] +=1 
    
    dist, angles = to_polar_coords(xpix,ypix)
    Rover.nav_angles = angles 
    mean_dir = np.mean(new_nav_angles)
    
    rock_map = find_rocks(warped, levels=(110,110,50))
    
    if rock_map.any():
        rock_x, rock_y = rover_coords(rock_map)
        rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
        rock_dist, rock_ang = to_polar_coords(rock_x,rock_y)
        rock_idx = np.argmin(rock_dist) #min dist rock pixel
        rock_xcen = rock_x_world[rock_idx]
        rock_ycen = rock_y_world[rock_idx]
    
        Rover.worldmap[rock_ycen, rock_xcen, 1] = 255
        Rover.vision_image[:,:,1] = rock_map * 255
    
    
    
    else:
        Rover.vision_image[:,:,1] = 0
       
    # debugger saves realtime images of autonomous mode in folder pipeline_realtime
    if debugger == True:
        fig = plt.figure(figsize=(12,9))
        plt.subplot(221)
        plt.imshow(image)
        plt.subplot(222)
        plt.imshow(warped)
        plt.subplot(223)
        plt.imshow(threshed, cmap='gray')
        plt.subplot(224)
        plt.plot(xpix, ypix, '.')
        plt.ylim(-160, 160)
        plt.xlim(0, 160)
        arrow_length = 100
        x_arrow = arrow_length * np.cos(mean_dir)
        y_arrow = arrow_length * np.sin(mean_dir)
        plt.arrow(0, 0, x_arrow, y_arrow, color='red', zorder=2, head_width=10, width=2)
        
        idx = np.random.randint(0, 999999999)
        fig.savefig('../pipeline_realtime/Image' + str(idx) + '.jpg')
        plt.close(fig)

 
    
    
    return Rover

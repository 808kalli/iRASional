import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
#from src.move.threads.movements.stanley import *
import logging


def canny(image):
    #used to extract the edges from the image
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 150, 255) #usually 1:2 or 1:3 on low threshod : high threshold
    return canny


def ROI(image):
    # sets a region of interest in the binary image and uses it as a mask
    height, width = image.shape
    polygons = np.array([[(0, height), (int(width/5), int(2*height/5)), (int(4*width/5), int(2*height/5)), (width, height)]]) 
    #matplotlib: height 0 on top and max on the lowest spot
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


def perspective_transform(cropped):
    height, width = cropped.shape
    x1 = 0
    x2 = width
    src=np.float32([[x1,int(9*height/10)],[x2,int(9*height/10)],[int(x2/5),int(2*height/5)],[int(4*x2/5),int(2*height/5)]])
    dst=np.float32([[x1,height],[x2,height],[x1,0],[x2,0]])
    M = cv2.getPerspectiveTransform(src, dst)  # M is our transformation matrix
    Minv = cv2.getPerspectiveTransform(dst, src)  # Minv is the inverse of M
    img_size=(width,height)
    warped = cv2.warpPerspective(cropped, M, img_size, flags=cv2.INTER_LINEAR)  # The transformed image
    return warped, Minv


def lane_det(bird_view, original_image, Minv):
    bird_height, bird_width = bird_view.shape
    right_lane_warning = 0
    left_lane_warning = 0
    
    #----- Creating Histogram - Finding where the lanes begin -----#
    
    #in final form pipeline will only need bird_view as arguement
    # After creating a warped binary Image,
    # Take a histogram of the bottom half of the image
    histogram = np.sum(bird_view[int((6*bird_height)/10):,:], axis=0)
    #we want to create the histogram close to the vehichle, so the error is small
    # print(histogram)
    # Create an output image to draw on and  visualize the result
    out_img = np.dstack((bird_view, bird_view, bird_view))*255
    # cv2.imshow("out", out_img)


    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = int(bird_width/2)

    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint #adding midpoint, histogram[midpoint:] does not use the first half of the histogram
    
    safe_pixs = int(bird_width/10) #safe number of pixels distance from the middle to be considered left or right lane
    
    if (rightx_base <= midpoint + safe_pixs or max(histogram[midpoint:])==0):
        # print("no right lane")
        right_lane_warning = 1
        
    if (leftx_base >= midpoint -safe_pixs or max(histogram[:midpoint])==0):
        # print("no left lane")
        left_lane_warning = 1
    
    
    #----- Sliding Windows - Finding Lanes -----#
    
    # Choose the number of sliding windows
    nwindows = 30

    # Set height of windows
    window_height = int(bird_height/nwindows)


    # Set the width of the windows +/- margin
    margin = int(bird_width/4)
    start_margin = margin

    # Set minimum number of pixels found to recenter window
    minpix = 5

    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = bird_view.nonzero()
    # tuple of arrays of nonzero elements (x, y)
    nonzeroy = np.array(nonzero[0])
    # array of y elements
    nonzerox = np.array(nonzero[1])
    # array of x elements

    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []
    # Step through the windows one by one
    left_warning = 0
    right_warning = 0
    for window in range(nwindows):
        if (left_lane_warning == 0):
        # Identify window boundaries in x and y
            win_y_low = bird_height - (window+1)*window_height
            win_y_high = bird_height - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
            (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
                out_img = cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0, 0, 255), 1)
            else:
                left_warning += 1


        if (right_lane_warning == 0):
            win_y_low = bird_height - (window+1)*window_height
            win_y_high = bird_height - window*window_height
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
            (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
            right_lane_inds.append(good_right_inds)
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))
                out_img = cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0, 0, 255), 1)
            else:
                right_warning += 1
        
        # check if the method cannot be trusted for any of the two lanes
        safe_windows_num = 15 #how many windows can we afford to lose
        if (right_warning>=safe_windows_num):
            right_lane_warning=1
        if (left_warning>=safe_windows_num):
            left_lane_warning=1
            
            
        # Draw the windows on the visualization image
        # cv2.imshow('windows', out_img)
        
        # visualize windows
        margin = int(margin-(0.8*start_margin/nwindows))
        #reduce margin , so after 30 iterations margin = 20% of the starting margin
        #margin need to be redused so when the window is far up in the image it wont detect both lanes in the same window
        #WARNING: LARGE nwindows (line 72) can result in margin dropping to 0 very quickly, so choose nwindows wisely

    ploty = np.linspace(0, bird_height-1, bird_height)
    if (left_lane_warning == 0):
        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        # Fit a second order polynomial to each
        left_fit = np.polyfit(lefty, leftx, 2)
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        pts_left = np.int_(np.array(np.transpose(np.vstack([left_fitx, ploty]))))
        
    
    if (right_lane_warning == 0):
        right_lane_inds = np.concatenate(right_lane_inds)
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        right_fit = np.polyfit(righty, rightx, 2)
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        pts_right = np.int_(np.array(np.transpose(np.vstack([right_fitx, ploty]))))

    #THESE RETURN POINTS OF LEFT AND RIGHT LANE RESPECTIVELY

    #============== VISUALIZATION ================================
    # UNCOMMENT THESE TO VISUALIZE RESULTS, AND return result AT THE END
    # Visualize Lane Curves
    lanes = np.zeros(bird_view.shape)
    warp_zero = np.zeros_like(bird_view).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    if (left_lane_warning == 0):
        for point in pts_left:
            lanes = cv2.circle(lanes,tuple(point),0,255)
            color_warp = cv2.circle(color_warp, tuple(point), 0, (0, 0, 255), 20)
    if (right_lane_warning == 0):
        for point in pts_right:
            lanes = cv2.circle(lanes,tuple(point),0,255)
            color_warp = cv2.circle(color_warp, tuple(point), 0, (255, 0, 0), 20)

    # visualize the desired trajectory for the car
    desired_trajectory = []
    desired_trajectory_x = []
    desired_trajectory_y = []
    degree_error = 0
    if (left_lane_warning == 0 and right_lane_warning == 0):
        desired_trajectory = (pts_right+pts_left)//2
        for point in desired_trajectory:
            desired_trajectory_x.append(point[0])
            desired_trajectory_y.append(point[1])
            lanes = cv2.circle(lanes,tuple(point),0,255)
            color_warp = cv2.circle(color_warp, tuple(point), 0, (128, 0, 128), 5)

        desired_fit = np.polyfit(desired_trajectory_y, desired_trajectory_x, 2)
        degree_error = 200 * desired_fit[0] + desired_fit[1]

    elif (left_lane_warning == 0):
        for point in pts_left:
            point = point + [bird_width//2, 0]
            desired_trajectory.append(point)
            desired_trajectory_x.append(point[0])
            desired_trajectory_y.append(point[1])
            lanes = cv2.circle(lanes,tuple(point),0,255)
            color_warp = cv2.circle(color_warp, tuple(point), 0, (0, 0, 255), 5)

        desired_fit = np.polyfit(desired_trajectory_y, desired_trajectory_x, 2)
        degree_error = 200 * desired_fit[0] + desired_fit[1]            
        # print("left only")
        
    elif (right_lane_warning == 0):
        for point in pts_right:
            point = point - [bird_width//2, 0]
            desired_trajectory.append(point)
            desired_trajectory_x.append(point[0])
            desired_trajectory_y.append(point[1])
            lanes = cv2.circle(lanes,tuple(point),0,255)
            color_warp = cv2.circle(color_warp, tuple(point), 0, (255, 0, 0), 5)

        desired_fit = np.polyfit(desired_trajectory_y, desired_trajectory_x, 2)
        #print(desired_fit)
        degree_error = 200 * desired_fit[0] + desired_fit[1]            
        # print("right only")
    else:
        print("NO LANES FOUND")

    # visualize current trajectory (if the car would go straight)
    current_trajectory = []
    for y in range(0, int(bird_height/3)):
        current_trajectory.append([int(bird_width/2), bird_height-y])
    #print(current_trajectory)
    for point in current_trajectory:
        lanes = cv2.circle(lanes,tuple(point),0,255)
        color_warp = cv2.circle(color_warp, tuple(point), 0, (0, 165, 255), 5)
    
    #show results 
    # cv2.imshow('lanes', lanes)

    # #Warp the blank back to original image space using inverse perspective matrix (Minv)
    newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))

    #Combine the result with the original image
    result = cv2.addWeighted(original_image, 0.8, newwarp, 1, 0)
    #result = cv2.resize(result, [1280, 960], interpolation = cv2.INTER_AREA)
    # cv2.imshow('result', result)
    
    error_in_pixs = current_trajectory[0][0] - desired_trajectory[bird_height-1][0]

    # print(degree_error)
    degree_error = np.arctan(degree_error) * (180 / np.pi)

    # print(f"Error in pixels is: {error_in_pixs}")
    # print(f"Error in degrees is {degree_error}")
    
    return error_in_pixs, degree_error

K = 20 #control gain 
max_steer = 22

def stanley_correction( crosstrack_error,heading_error,velocity): #crosstrack in pixels
    desired_steering_angle = -heading_error - 57*np.arctan(K*crosstrack_error/velocity)
    limited_steering_angle = np.clip(desired_steering_angle, -max_steer, max_steer)
    limited_steering_angle = round(limited_steering_angle,1)
    return limited_steering_angle

def followLane(img):
    try:
        # print(type(img))
        canny_img = canny(img)
        cropped_image = ROI(canny_img)
        bird, Minv = perspective_transform(cropped_image)
        error_in_pixs, error_in_degrees =lane_det(bird, img, Minv)
        print(f"Error in pixels is: {error_in_pixs}")
        print(f"Error in degrees is {error_in_degrees}")
        return stanley_correction(error_in_pixs, error_in_degrees, 10)
    except:
        # logging.error("error", exc_info = True)
        print("no lanefollowing")
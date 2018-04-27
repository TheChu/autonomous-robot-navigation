################################################################################
##############################  NOTES  #######################################
################################################################################
"""
Developed by Charlee Van Eijk and David Chu

Parts of code from the following were used to detect color:
https://www.pyimagesearch.com/2016/10/31/detecting-multiple-bright-spots-in-an-image-with-python-and-opencv/
"""
################################################################################
##############################  IMPORTS  #######################################
################################################################################

# Necessary imports
import numpy as np
import time
import sys
import os
import cv2
import requests
from copy import copy, deepcopy
import imutils
from imutils import contours
from skimage import measure

################################################################################
#############################  PARAMETERS  #####################################
################################################################################

# Parameters
THRESHOLD = 140
THRESHOLD_CORNERS = 20
MAX_VALUE = 255
DEBUG_CROP = False
DEBUG  = False
DEBUG_THRESHOLD = False
DEBUG_THRESHOLD_MILLIS = 20000
DEBUG_CORRECT = False
MAZE_CELLS_WIDTH = 5
MAZE_CELLS_HEIGHT = 4
WALL_INTENSITY_THRESHOLD = 75
STRIP_WIDTH_FACTOR = 10
STRIP_HEIGHT_FACTOR = 10

url = "http://134.173.24.86:8080/shot.jpg"

RED_BOUND_LOWER = [5, 5, 75]
RED_BOUND_UPPER = [70, 70, 250]

ACTUAL_MAZE = [[[1, 0, 0, 1], [1, 1, 0, 0], [1, 0, 0, 1], [1, 0, 0, 0], [1, 1, 0, 0]],
               [[0, 1, 0, 1], [0, 1, 0, 1], [0, 1, 0, 1], [0, 1, 0, 1], [0, 1, 0, 1]],
               [[0, 1, 0, 1], [0, 0, 1, 1], [0, 1, 0, 0], [0, 1, 0, 1], [0, 1, 0, 1]],
               [[0, 0, 1, 1], [1, 0, 1, 0], [0, 1, 1, 0], [0, 1, 1, 1], [0, 1, 1, 1]]]

################################################################################
##########################  HELPER FUNCTIONS  ##################################
################################################################################

def calibrateIsolatedImage(img):
    # Store height, width of image in pixels
    imgCopy = img.copy()
    img_pixel_height, img_pixel_width, _ = img.shape

    # Store cell width and height in pixels
    cell_pixel_width = img_pixel_width/MAZE_CELLS_WIDTH
    cell_pixel_height = img_pixel_height/MAZE_CELLS_HEIGHT

    # Initialize boundary points
    x_index = 0
    y_index = 0
    x_right_bound = x_index + cell_pixel_width
    y_upper_bound = y_index + cell_pixel_height

    # Store ROI boundary parameters
    sub_img_ex = img[y_index:y_upper_bound, x_index:x_right_bound]
    cell_pixel_height, cell_pixel_width, _ = sub_img_ex.shape
    horizontal_roi_height = cell_pixel_height/STRIP_WIDTH_FACTOR
    vertical_roi_width = cell_pixel_width/STRIP_HEIGHT_FACTOR
    roi_top_y_index = cell_pixel_height - horizontal_roi_height
    roi_right_x_index = cell_pixel_width - vertical_roi_width

    y_sep = cell_pixel_height - 2 * horizontal_roi_height
    x_sep = cell_pixel_width - 2 * vertical_roi_width

    # Iterate through cells, move boundary points, display segmented image
    for row in range(MAZE_CELLS_HEIGHT):
        for col in range(MAZE_CELLS_WIDTH):
            # bottom
            cv2.rectangle(img, (x_index, y_index + y_sep), (x_index + cell_pixel_width, y_index + y_sep), (0,0,255), 2)
            # top
            cv2.rectangle(img, (x_index, y_index + horizontal_roi_height), (x_index + cell_pixel_width, y_index + horizontal_roi_height), (0,0,255), 2)
            # right
            cv2.rectangle(img, (x_index + cell_pixel_width - vertical_roi_width, y_index), (x_index + cell_pixel_width, y_index + cell_pixel_height), (0,0,255), 2)
            # left
            cv2.rectangle(img, (x_index, y_index), (x_index + vertical_roi_width, y_index + cell_pixel_height), (0,0,255), 2)

            cv2.rectangle(img, (x_index, y_index), (x_right_bound, y_upper_bound), (255,0,0), 2)
            x_index = x_right_bound
            x_right_bound = x_index + cell_pixel_width
        x_index = 0
        x_right_bound = x_index + cell_pixel_width
        y_index = y_upper_bound
        y_upper_bound = y_index + cell_pixel_height

    # Display overlay maze
    cv2.namedWindow('overlay', cv2.WINDOW_NORMAL)
    imS = cv2.resize(img, (1280,680))
    cv2.imshow('overlay', imS)

    # Display gray image and thresholded
    cv2.namedWindow('gray', cv2.WINDOW_NORMAL)
    cv2.namedWindow('thresh', cv2.WINDOW_NORMAL)
    cv2.namedWindow('gauss', cv2.WINDOW_NORMAL)
    im_gray =  cv2.cvtColor(imgCopy, cv2.COLOR_BGR2GRAY)
    imSgray = cv2.resize(im_gray, (640,340))
    gauss = cv2.GaussianBlur(im_gray,(5,5),1)
    imSgauss = cv2.resize(gauss, (640,340))
    ret,thresh = cv2.threshold(gauss,THRESHOLD,MAX_VALUE,cv2.THRESH_BINARY_INV)
    imSthresh = cv2.resize(thresh, (640,340))
    cv2.imshow('gray', imSgray)
    cv2.imshow('thresh', imSthresh)
    cv2.imshow('gauss', imSgauss)

    k = cv2.waitKey(1) & 0xff

"""
Function: isolateMaze
In: none
Out: image of roi that is maze
Description: Extracts a maze from a picture
"""
def isolateMaze():
    i = 0
    while i < 3 or DEBUG_CROP:
        # Get webcam video
        img_resp = requests.get(url)
        img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
        img = cv2.imdecode(img_arr, -1)
        imgCopy = img.copy()

        # Apply red mask
        lower_red = np.array(RED_BOUND_LOWER, dtype = "uint8")
        upper_red = np.array(RED_BOUND_UPPER, dtype = "uint8")
    	mask = cv2.inRange(img, lower_red, upper_red)
    	red = cv2.bitwise_and(img, img, mask = mask)

        if DEBUG_CROP:
            imS = cv2.resize(red, (640,340))
            cv2.imshow('overlay', imS)

        im_gray =  cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
        if DEBUG_CROP:
            imG = cv2.resize(im_gray, (640,340))
            cv2.imshow('gray', imG)

        ret,thresh = cv2.threshold(im_gray,THRESHOLD_CORNERS, MAX_VALUE, cv2.THRESH_BINARY)
        if DEBUG_CROP:
            imT = cv2.resize(thresh, (640,340))
            cv2.imshow('crop', imT)

        if DEBUG_CROP:
            cv2.imshow('orig', imgCopy)

        k = cv2.waitKey(1) & 0xff

        # perform a connected component analysis on the thresholded
        # image, then initialize a mask to store only the "large"
        # components
        labels = measure.label(thresh, neighbors=8, background=0)
        mask = np.zeros(thresh.shape, dtype="uint8")

        # loop over the unique components
        for label in np.unique(labels):
        	# if this is the background label, ignore it
        	if label == 0:
        		continue

        	# otherwise, construct the label mask and count the
        	# number of pixels
        	labelMask = np.zeros(thresh.shape, dtype="uint8")
        	labelMask[labels == label] = 255
        	numPixels = cv2.countNonZero(labelMask)

        	# if the number of pixels in the component is sufficiently
        	# large, then add it to our mask of "large blobs"
        	if numPixels > 300:
        		mask = cv2.add(mask, labelMask)

        # find the contours in the mask, then sort them from left to
        # right
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        	cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        cnts = contours.sort_contours(cnts)[0]

        corners = []
        # loop over the contours
        for (i, c) in enumerate(cnts):
            # draw the bright spot on the image
            (x, y, w, h) = cv2.boundingRect(c)
            ((cX, cY), radius) = cv2.minEnclosingCircle(c)
            cv2.circle(imgCopy, (int(cX), int(cY)), int(radius), (0, 0, 255), 3)
            corners.append([int(cX), int(cY)])

        # show the output image
        if DEBUG_CROP:
            imC = cv2.resize(imgCopy, (640,340))
            cv2.imshow("Orig", imC)

        # show the roi image
        if (len(corners) == 3):
            roi = imgCopy[corners[0][1]:corners[2][1], corners[0][0]:corners[2][0]]
        else:
            roi = imgCopy[corners[0][1]:corners[1][1], corners[0][0]:corners[1][0]]

        if DEBUG_CROP:
            imR = cv2.resize(roi, (640,340))
            cv2.imshow("Cropped", imR)

        k = cv2.waitKey(1) & 0xff

        i += 1
        if i == 2 and not DEBUG_CROP:
            return roi, corners

"""
Function: photoMaze
In: none
Out: thresholded image (numpy array)
Description: Takes a picture of the maze, converts it to grayscale, thresholds it
"""
def photoMaze(img):
    i = 0
    while i < 3 or DEBUG:

        if (DEBUG):                       # For debugging, show image on screen
            calibrateIsolatedImage(img)

        # Continually apply filter to camera
        im_gray =  cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(im_gray,THRESHOLD,MAX_VALUE,cv2.THRESH_BINARY_INV)
        gauss = cv2.GaussianBlur(thresh,(5,5),1)

        i += 1
        if i == 2 and not DEBUG:
            return gauss

"""
Function: discretizeMaze
In: image of maze
Out: list of sub images (numpy array)
Description: Splits image into MAZE_WIDTH * MAZE_HEIGHT sub images
"""
def discretizeMaze(maze_img):
    # Store height, width of image in pixels
    img_pixel_height, img_pixel_width = maze_img.shape

    # Initialize 2d array to store segmented images
    img_arr = [[0]*MAZE_CELLS_WIDTH for _ in range(MAZE_CELLS_HEIGHT)]

    # Store cell width and height in pixels
    cell_pixel_width = img_pixel_width/MAZE_CELLS_WIDTH
    cell_pixel_height = img_pixel_height/MAZE_CELLS_HEIGHT

    # Initialize boundary points
    x_index = 0
    y_index = 0
    x_right_bound = x_index + cell_pixel_width
    y_upper_bound = y_index + cell_pixel_height

    # Iterate through cells, move boundary points, store segmented image
    i = 0
    for row in range(MAZE_CELLS_HEIGHT):
        for col in range(MAZE_CELLS_WIDTH):
            sub_img = maze_img[y_index:y_upper_bound, x_index:x_right_bound]
            img_arr[row][col] = sub_img
            x_index = x_right_bound
            x_right_bound = x_index + cell_pixel_width
            i += 1
        x_index = 0
        x_right_bound = x_index + cell_pixel_width
        y_index = y_upper_bound
        y_upper_bound = y_index + cell_pixel_height

    return img_arr

"""
Function: isWall
In: image of cell segment
Out: Boolean
Description: Returns a boolean based on whether the average pixel intensity is
             above a specific threshold (aka whether there is a wall there)
"""
def isWall(strip_img):
    return int(cv2.mean(strip_img)[0] > WALL_INTENSITY_THRESHOLD)

"""
Function: imageToCell
In: image of cell
Out: 4 element list
Description: Returns a representation of the cell. 1 hot encoding for walls
             corresponding to N E S W. eg. [0 0 1 0] means a wall to the south.
"""
def imageToCell(img, row, col):

    # Array to store segmented images
    strips = []

    # Store ROI boundary parameters
    cell_pixel_height, cell_pixel_width = img.shape
    horizontal_roi_height = cell_pixel_height/STRIP_HEIGHT_FACTOR
    vertical_roi_width = cell_pixel_width/STRIP_WIDTH_FACTOR

    # Get top, right, bottom, and left ROIs
    roi_top = img[0:horizontal_roi_height, 0:cell_pixel_width]
    strips.append(roi_top)

    roi_right_x_index = cell_pixel_width - vertical_roi_width
    roi_right = img[0:cell_pixel_height, roi_right_x_index:roi_right_x_index + vertical_roi_width]
    strips.append(roi_right)

    roi_bottom_y_index = cell_pixel_height - horizontal_roi_height
    roi_bottom = img[roi_bottom_y_index:cell_pixel_height, 0:cell_pixel_width]
    strips.append(roi_bottom)

    roi_left = img[0:cell_pixel_height, 0:vertical_roi_width]
    strips.append(roi_left)

    if DEBUG_THRESHOLD:
        cv2.imshow('cell', img)
        for i in range(len(strips)):
            cv2.imshow('strip ' + str(i), strips[i])
        cell = [int(cv2.mean(i)[0] > WALL_INTENSITY_THRESHOLD) for i in strips]
        print str((row, col)), "thresh: ", cell, "actual: ", ACTUAL_MAZE[row][col], [cv2.mean(i)[0] for i in strips], cell == ACTUAL_MAZE[row][col]
        if cell != ACTUAL_MAZE[row][col]:
            k = cv2.waitKey(DEBUG_THRESHOLD_MILLIS) & 0xff

    cell = [int(cv2.mean(i)[0] > WALL_INTENSITY_THRESHOLD) for i in strips]

    return cell

"""
Function: digitizeMaze
In: sub_imgs (list of images)
Out: 2d array
Description: Returns a representation of the array by analyzing each cell
"""
def digitizeMaze(sub_imgs):
    # Initialize 2d array to store maze representation
    maze_arr = [[0]*MAZE_CELLS_WIDTH for _ in range(MAZE_CELLS_HEIGHT)]

    # Iterate through 2d image array, turn each into a cell representation
    for row in range(MAZE_CELLS_HEIGHT):
        for col in range(MAZE_CELLS_WIDTH):
            maze_arr[row][col] = imageToCell(sub_imgs[row][col], row, col)

    if DEBUG_CORRECT:
        print
        name = 'maze_arr in digitizeMaze'
        printMaze(name, maze_arr)
        checkMaze(name, maze_arr)

    return maze_arr

"""
Function: correctMaze
In: 2d array maze representation
Out: 2d array corrected
Description: Corrects for missed walls
"""
def correctMaze(maze_in):
    maze = deepcopy(maze_in)
    h = len(maze)
    w = len(maze[0])
    for row in range(h - 1):
        for col in range(w - 1):
            if maze[row][col][3] or maze[row][col + 1][1]:
                maze[row][col][3] = maze[row][col + 1][1] = 1
            if maze[row][col][2] or maze[row + 1][col][0]:
                maze[row][col][2] = maze[row + 1][col][0] = 1
    if maze[h - 1][w - 1][1] or maze[h - 1][w - 2][3]:
        maze[h - 1][w - 1][1] = maze[h - 1][w - 2][3] = 1
    if maze[h - 1][w - 1][0] or maze[h - 2][w - 1][2]:
        maze[h - 1][w - 1][0] = maze[h - 2][w - 1][2] = 1
    return maze

"""
Function: checkMaze
In: 2d array maze representation
Out: None
Description: Prints whether the output maze matches the actual maze, along with
             how many cells don't match.
"""
def checkMaze(maze_name, maze):
    wrong_walls = {}
    for row in range(MAZE_CELLS_HEIGHT):
        for col in range(MAZE_CELLS_WIDTH):
            if maze[row][col] != ACTUAL_MAZE[row][col]:
                wrong_walls[(row, col)] = str(maze[row][col]) + ' vs actual ' + str(ACTUAL_MAZE[row][col])
    correct = len(wrong_walls) == 0
    print "Captured ", maze_name,  " correctly? ", correct
    print "Number of wrong cells: ", len(wrong_walls)
    print "List of wrong walls: "
    for x in wrong_walls:
        print (x)
        print wrong_walls[x]
    print

"""
Function: printMaze
In: 2d array maze representation
Out: None
Description: Returns a boolean depending on if the output maze matches the actual
             maze. If false returns a list of missed cells with correspnding missed
             walls.
"""
def printMaze(maze_name, maze):
    print maze_name
    for row in range(MAZE_CELLS_HEIGHT):
            print maze[row]
    print

################################################################################
################################  MAIN  ########################################
################################################################################

def getMaze():

    maze_img, corners = isolateMaze()      # Crop maze, get bot location

    fltr_maze_img = photoMaze(maze_img)    # Filter maze image

    if (DEBUG):                            # For debugging, show image on screen
        cv2.imshow('Threshold', fltr)
        k = cv2.waitKey(5000) & 0xff

    sub_imgs = discretizeMaze(fltr_maze_img)# Split image into cell images

    if (DEBUG):                            # For debugging, show segmented images
        i = 0
        for row in range(MAZE_CELLS_HEIGHT):
            for col in range(MAZE_CELLS_WIDTH):
                cv2.imshow('Segmented image: ' + str(i), sub_imgs[row][col])
                i += 1
        k = cv2.waitKey(30000) & 0xff

    grid_arr = digitizeMaze(sub_imgs)   # Turn maze into array

    #corr_arr = correctMaze(grid_arr)    # Correct 2d array for misalignments #TODO

    if DEBUG_CORRECT:
        printMaze('ACTUAL', ACTUAL_MAZE)
        printMaze('RAW', grid_arr)
        printMaze('CORRECTED', corr_arr)
        checkMaze('RAW', grid_arr)
        checkMaze('CORRECTED', corr_arr)

    cv2.destroyAllWindows()

    return grid_arr, corners

# if __name__ == '__main__':
#   main()

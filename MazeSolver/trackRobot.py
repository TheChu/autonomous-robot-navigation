################################################################################
##############################  NOTES  #######################################
################################################################################
"""
Developed by Charlee Van Eijk and David Chu

This file contains all necessary functions for getting the x,y,theta of the bot
in pixels and radians, respectively.
"""
################################################################################
##############################  IMPORTS  #######################################
################################################################################

# Necessary imports
from captureMaze import *

################################################################################
#############################  PARAMETERS  #####################################
################################################################################

# Parameters
DEBUG_WEBCAM = False
DEBUG_PHOTO = False
DEBUG_FILTER = False
DEBUG_BRIGHT = False
DEBUG_STATE = False
DEBUG_LOCALIZE = False
DEBUG_BLUE = False
DEBUG_RED = True
TRACKING_THRESHOLD = 140
BLUE_GRAY_THRESHOLD = 40
CIRCLE_DIAMETER_PIXELS = 50
TRACKING_NUMPIXELS_THRESHOLD = 200
#                   G   B   R
BLUE_BOUND_LOWER = [60, 20, 0]
BLUE_BOUND_UPPER = [255, 255, 90]

RED_BOUND_LOWER = [5, 5, 75]
RED_BOUND_UPPER = [70, 70, 250]

# SimpleBlobDetector parameters.
PARAMS = cv2.SimpleBlobDetector_Params()

################################################################################
##########################  HELPER FUNCTIONS  ##################################
################################################################################
"""
Function: webcamTest
In: none
Out: none
Description: Displays image captured by webcam. Used to verify setup is correct.
"""
def webcamTest():
    # Get webcam video
    img_resp = requests.get(url)
    img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
    img = cv2.imdecode(img_arr, -1)
    img_rsz = cv2.resize(img, (640,340))
    cv2.imshow('Raw', img_rsz)
    k = cv2.waitKey(1) & 0xff

"""
Function: photoRobot
In: none
Out: webcam image
Description: Returns a non-cropped picture of the maze
"""
def photoBot():
    # Get webcam video
    img_resp = requests.get(url)
    img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
    img = cv2.imdecode(img_arr, -1)
    if DEBUG_PHOTO:
        img_rsz = cv2.resize(img, (640,340))
        cv2.imshow('Raw', img_rsz)
        k = cv2.waitKey(1) & 0xff
    return img

"""
Function: filterFrame
In: webcam image
Out: thresholded image
Description: Takes a picture of the maze, converts it to grayscale, thresholds it,
             and applies a gaussian blur.
"""
def filterFrame(img, corners):
    # img -> grayscale -> thresholded -> gaussian
    img_crop = img[corners[0][1]:corners[-1][1], corners[0][0]:corners[-1][0]]
    im_gray =  cv2.cvtColor(img_crop, cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(im_gray,TRACKING_THRESHOLD,MAX_VALUE,cv2.THRESH_BINARY_INV)
    gauss = cv2.GaussianBlur(thresh,(5,5),1)
    if DEBUG_FILTER:
        img_rsz = cv2.resize(gauss, (640,340))
        cv2.imshow('Filtered', img_rsz)
        k = cv2.waitKey(1) & 0xff
    return img_crop, gauss

"""
Function: removeBlue
In: thresholded maze image
Out: maze image without walls
Description: Removes the blue walls from maze image
"""
def removeBlue(color_img, thresh_img):
    # Apply blue mask
    lower_blue = np.array(BLUE_BOUND_LOWER, dtype = "uint8")
    upper_blue = np.array(BLUE_BOUND_UPPER, dtype = "uint8")
    mask = cv2.inRange(color_img, lower_blue, upper_blue)
    blue_img = cv2.bitwise_and(color_img, color_img, mask = mask)
    im_gray =  cv2.cvtColor(blue_img, cv2.COLOR_BGR2GRAY)
    ret,blue_thresh = cv2.threshold(im_gray,BLUE_GRAY_THRESHOLD,MAX_VALUE,cv2.THRESH_BINARY)
    thresh_img[np.where(blue_thresh == [255])] = [0]
    if DEBUG_BLUE:
        blue_img_rsz = cv2.resize(blue_img, (640,340))
        cv2.imshow('Blue', blue_img_rsz)
        im_gray_rsz = cv2.resize(im_gray, (640, 340))
        cv2.imshow('Blue to Gray', im_gray_rsz)
        blue_thresh_rsz = cv2.resize(blue_thresh, (640, 340))
        cv2.imshow('Blue Thresholded', blue_thresh_rsz)
        thresh_img_rsz = cv2.resize(thresh_img, (640,340))
        cv2.imshow('Without Blue', thresh_img_rsz)
        k = cv2.waitKey(1) & 0xff
    return thresh_img


"""
Function: findRedSpot
In: image
Out: array of red coordinate positions on bot
Description: Finds red objects in image, stores their positions, returns thresh_img
             in an array.
"""
def findRedSpot(img):
    # Apply red mask
    lower_red = np.array(RED_BOUND_LOWER, dtype = "uint8")
    upper_red = np.array(RED_BOUND_UPPER, dtype = "uint8")
    mask = cv2.inRange(img, lower_red, upper_red)
    red = cv2.bitwise_and(img, img, mask = mask)
    imgCopy = img.copy()

    if DEBUG_RED:
        img_rsz = cv2.resize(img, (640, 340))
        cv2.imshow('og', img_rsz)
        imS = cv2.resize(red, (640,340))
        cv2.imshow('overlay', imS)
        k = cv2.waitKey(1) & 0xff

    im_gray =  cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)

    ret,thresh = cv2.threshold(im_gray,THRESHOLD_CORNERS, MAX_VALUE, cv2.THRESH_BINARY)


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

    bot_spots = [[0,0],[0,0]]
    # loop over the contours
    for (i, c) in enumerate(cnts):
        # draw the bright spot on the image
        (x, y, w, h) = cv2.boundingRect(c)
        ((cX, cY), radius) = cv2.minEnclosingCircle(c)
        if radius > RADIUS_CUTOFF_PIXELS:
            bot_spots[0] = [int(cX), int(cY)]
        elif cX > 10 and cX < 1280:
            bot_spots[1] = [int(cX), int(cY)]
        cv2.circle(imgCopy, (int(cX), int(cY)), int(radius), (0, 0, 255), 3)

    # show the output image
    if DEBUG_RED:
        imC = cv2.resize(imgCopy, (640,340))
        cv2.imshow("Orig", imC)
        k = cv2.waitKey(1) & 0xff

    return bot_spots

def angle_wrap(a):
    while a > pi:
        a = a - 2*pi
    while a < -pi:
        a = a + 2*pi
    return a


"""
Function: getState
In: array of coordinates
Out: state array
Description: Calculates bot orientation given coordinates of two reference pointsself.
             Returns one of the coordinates and angle.
"""
def getState(bot_spots):
    x_dist = bot_spots[0][0] - bot_spots[1][0]
    y_dist = bot_spots[0][1] - bot_spots[1][1]
    angle = angle_wrap(((atan2(y_dist, x_dist)) - pi) * -1)
    if DEBUG_STATE:
        print bot_spots
        print x_dist, y_dist
        print degrees(angle)
    state = (bot_spots[0][0], bot_spots[0][1], angle)
    return state

"""
Function: localizeBot
In: preprocessed maze image
Out: list of robot x, y, and theta
Description: Returns the robot state
"""
def localizeBot(color, thresh):
    state = [0,0,0]
    bot_spots = findRedSpot(color)
    state = getState(bot_spots)
    if DEBUG_LOCALIZE:
        print state
        print
    return state

################################################################################
################################  MAIN  ########################################
################################################################################

def main():

    while(DEBUG_WEBCAM):
        webcamTest()

    grid_arr, corners, bot_spots = getMaze()

    while(1):
        frame = photoBot()                           # Get image from webcam
        color, thresh = filterFrame(frame, corners)  # Crops and thresholds image
        [x, y, theta] = localizeBot(color, thresh)   # Get x, y, and theta
        print x, y, degrees(theta)

if __name__ == '__main__':
  main()

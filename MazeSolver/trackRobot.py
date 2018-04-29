################################################################################
##############################  NOTES  #######################################
################################################################################
"""
In the process of tracking bot on blue-less image
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
DEBUG_LOCALIZE = False
DEBUG_BLUE = False
DEBUG_RED = False
TRACKING_THRESHOLD = 140
BLUE_GRAY_THRESHOLD = 40
CIRCLE_DIAMETER_PIXELS = 50
TRACKING_NUMPIXELS_THRESHOLD = 200
#                   G   B   R
BLUE_BOUND_LOWER = [60, 20, 0]
BLUE_BOUND_UPPER = [255, 255, 90]

#                   G   B   R
GREEN_BOUND_LOWER = [100, 0, 0]
GREEN_BOUND_UPPER = [255, 50, 100]

RED_BOUND_LOWER = [5, 5, 75]
RED_BOUND_UPPER = [70, 70, 250]

# SimpleBlobDetector parameters.
PARAMS = cv2.SimpleBlobDetector_Params()

# Change thresholds

PARAMS.minThreshold = 0;
PARAMS.maxThreshold = 255;

# Filter by Area.
PARAMS.filterByArea = True
PARAMS.minArea = 20
PARAMS.maxArea= 100

# Filter by Circularity
PARAMS.filterByCircularity = True
PARAMS.minCircularity = 0.7
PARAMS.maxCircularity = 1

# Filter by Convexity
PARAMS.filterByConvexity = False
PARAMS.minConvexity = 0.87
PARAMS.maxConvexity = 1

# Filter by Inertia
PARAMS.filterByInertia = False
PARAMS.minInertiaRatio = 0.5
PARAMS.maxInertiaRatio = 1

################################################################################
##########################  HELPER FUNCTIONS  ##################################
################################################################################

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
    img_crop = img[corners[0][1]:corners[1][1], corners[0][0]:corners[1][0]]
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
Function:
In:
Out:
Description:
"""
def findBrightestSpot(img):
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(img)
    image = img.copy()
    cv2.circle(image, maxLoc, CIRCLE_DIAMETER_PIXELS, (255, 0, 0), 2)
    if DEBUG_BRIGHT:
        image_rsz = cv2.resize(image, (640, 340))
        cv2.imshow('Brightest Spot', image_rsz)
        k = cv2.waitKey(1) & 0xff
    print (minVal, maxVal, minLoc, maxLoc)

def findCircle(img):
    output = img.copy()
    # detect circles in the image
    circles = cv2.HoughCircles(img, cv2.cv.CV_HOUGH_GRADIENT, 1.2, 10)

    # ensure at least some circles were found
    if circles is not None:
    	# convert the (x, y) coordinates and radius of the circles to integers
    	circles = np.round(circles[0, :]).astype("int")

    	# loop over the (x, y) coordinates and radius of the circles
    	for (x, y, r) in circles:
    		# draw the circle in the output image, then draw a rectangle
    		# corresponding to the center of the circle
    		cv2.circle(output, (x, y), r, (0, 255, 0), 4)

    	# show the output image
        image_rsz = cv2.resize(img, (640, 340))
    	cv2.imshow("output", image_rsz)
        k = cv2.waitKey(1) & 0xff

    return image_rsz

def findBlob(img):
    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    # if int(ver[0]) < 3 :
    #     detector = cv2.SimpleBlobDetector(PARAMS)
    # else :
    #     detector = cv2.SimpleBlobDetector_create(PARAMS)
    detector = cv2.SimpleBlobDetector()

    # Detect blobs.
    keypoints = detector.detect(img)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Show keypoints
    iwk_rsz = cv2.resize(im_with_keypoints, (640,340))
    cv2.imshow("Keypoints", iwk_rsz)
    k = cv2.waitKey(1) & 0xff

def findContour(img):
    # perform a connected component analysis on the thresholded
    # image, then initialize a mask to store only the "large"
    # components
    labels = measure.label(img, neighbors=8, background=0)
    print len(labels)
    mask = np.zeros(img.shape, dtype="uint8")

    # loop over the unique components
    for label in np.unique(labels):
        # if this is the background label, ignore it
        if label == 0:
            continue

        # otherwise, construct the label mask and count the
        # number of pixels
        labelMask = np.zeros(img.shape, dtype="uint8")
        labelMask[labels == label] = 255
        numPixels = cv2.countNonZero(labelMask)

        # if the number of pixels in the component is sufficiently
        # large, then add it to our mask of "large blobs"
        if numPixels > TRACKING_NUMPIXELS_THRESHOLD:
            mask = cv2.add(mask, labelMask)

    # find the contours in the mask, then sort them from left to
    # right
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    cnts = contours.sort_contours(cnts)[0]

    bot = []
    # loop over the contours
    for (i, c) in enumerate(cnts):
        # draw the bright spot on the image
        (x, y, w, h) = cv2.boundingRect(c)
        ((cX, cY), radius) = cv2.minEnclosingCircle(c)
        cv2.circle(img, (int(cX), int(cY)), int(radius), (0, 0, 255), 3)
        bot.append([int(cX), int(cY)])

    print bot
    # show the output image
    if DEBUG_CROP:
        img_rsz = cv2.resize(img, (640,340))
        cv2.imshow("Tracked", img_rsz)

def findRedSpot(img):
    # Apply red mask
    lower_red = np.array(RED_BOUND_LOWER, dtype = "uint8")
    upper_red = np.array(RED_BOUND_UPPER, dtype = "uint8")
    mask = cv2.inRange(img, lower_red, upper_red)
    red = cv2.bitwise_and(img, img, mask = mask)

    if DEBUG_CROP:
        imS = cv2.resize(red, (640,340))
        cv2.imshow('overlay', imS)

    im_gray =  cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)

    ret,thresh = cv2.threshold(im_gray,THRESHOLD_CORNERS, MAX_VALUE, cv2.THRESH_BINARY)

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

    red_spots = []
    # loop over the contours
    for (i, c) in enumerate(cnts):
        # draw the bright spot on the image
        (x, y, w, h) = cv2.boundingRect(c)
        ((cX, cY), radius) = cv2.minEnclosingCircle(c)
        cv2.circle(imgCopy, (int(cX), int(cY)), int(radius), (0, 0, 255), 3)
        red_spots.append([int(cX), int(cY)])

    print red_spots

    # show the output image
    if DEBUG_RED:
        imC = cv2.resize(imgCopy, (640,340))
        cv2.imshow("Orig", imC)

    k = cv2.waitKey(1) & 0xff

def angle_wrap(a):
    while a > pi:
        a = a - 2*pi
    while a < -pi:
        a = a + 2*pi
    return a

"""
Function: localizeBot
In: preprocessed maze image
Out: list of robot x, y, and theta
Description: Returns the robot state
"""
def localizeBot(color, thresh):
    #no_blue = removeBlue(color, thresh)
    #circled = findBrightestSpot(no_blue) #No luck
    #circled = findCircle(no_blue)        #No luck
    #circled = findBlob(no_blue)          #No luck
    #circled = findContour(no_blue)       #No luck
    # circled = findRedSpot(color)        #Not Necessary
    _, _, bot_spots = getMaze()
    x_dist = bot_spots[0][0] - bot_spots[1][0]
    y_dist = bot_spots[0][1] - bot_spots[1][1]
    angle = angle_wrap(((atan2(y_dist, x_dist)) - pi) * -1)
    if DEBUG_LOCALIZE:
        print bot_spots
        print x_dist, y_dist
        print degrees(angle)
    state = [bot_spots[0][0], bot_spots[0][1], angle]
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

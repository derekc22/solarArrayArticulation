import cv2 as cv

params = cv.SimpleBlobDetector_Params()

params.minThreshold = 0
params.maxThreshold = 255

params.filterByArea = False
params.minArea = 100
params.maxArea = 600

params.filterByColor = False
params.blobColor = 255

params.filterByCircularity = True
params.minCircularity = 0
params.maxCircularity = 1

params.filterByInertia = True
params.minInertiaRatio = 0
params.maxInertiaRatio = 1

params.filterByConvexity = True
params.minConvexity = 0
params.maxConvexity = 1

detector = cv.SimpleBlobDetector_create(params)

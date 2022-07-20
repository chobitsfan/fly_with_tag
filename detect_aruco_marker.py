# import the OpenCV library for computer vision
import cv2
import numpy as np

cv2.namedWindow('image_display', cv2.WINDOW_AUTOSIZE)

# Load the dictionary that was used to generate the markers.
# There's different aruco marker dictionaries, this code uses 6x6
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

# Initialize the detector parameters using default values
parameters = cv2.aruco.DetectorParameters_create()

# initialize the webcam as "camera" object
camera = cv2.VideoCapture("rtsp://192.168.0.34:8554/unicast")

# loop that runs the program forever
# at least until the "q" key is pressed
while True:

    # creates an "img" var that takes in a camera frame
    ret, img = camera.read()
    if ret:
        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # detect aruco tags within the frame
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

        rvecs , tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 0.2, np.array([[978.05583, 0, 644.3227087], [0, 980.40099676, 377.5166175], [0, 0, 1]]), 
        np.array([0.1542403792, -0.207237266, 0.00062078491848, 0.00089082489, -0.2166006565]))

        # draw box around aruco marker within camera frame
        img = cv2.aruco.drawDetectedMarkers(img, markerCorners, markerIds)

        # if a tag is found...
        if markerIds is not None:
            # for every tag in the array of detected tags...
            for i in range(len(markerIds)):
                r_mtx = cv2.Rodrigues(rvecs[0][0])
                r_mtx = r_mtx[0].transpose()
                print(markerIds[0], np.matmul(r_mtx, np.negative(tvecs[0][0])))

                # get the center point of the tag
                #center = markerCorners[i][0]
                #M = cv2.moments(center)
                #cX = int(M["m10"] / M["m00"])
                #cY = int(M["m01"] / M["m00"])
                # draws a red dot on the marker center
                #cv2.circle(img, (cX, cY), 1, (0, 0, 255), 8)
                # writes the coordinates of the center of the tag
                #cv2.putText(img, str(cX) + "," + str(cY), (cX + 40, cY - 40), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
                #(topLeft, topRight, bottomRight, bottomLeft) = markerCorners[i][0]
                # convert each of the (x, y)-coordinate pairs to integers
                #topRight = (int(topRight[0]), int(topRight[1]))
                #bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                #bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                #topLeft = (int(topLeft[0]), int(topLeft[1]))

                # draw the bounding box of the ArUCo detection
                #cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
                #cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
                #cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
                #cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)

        # Display the resulting frame
        cv2.imshow('image_display', img)

    # handler to press the "q" key to exit the program
    if cv2.waitKey(10) & 0xff == 113 :
        print("bye")
        break

# When everything done, release the capture
camera.release()
cv2.destroyAllWindows()
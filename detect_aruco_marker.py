# import the OpenCV library for computer vision
import cv2, time, math
import numpy as np
from pymavlink import mavutil
from pyquaternion import Quaternion
from dt_apriltags import Detector

def rotationMatrixToEulerAngles(R) :
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

cv2.namedWindow('image_display', cv2.WINDOW_AUTOSIZE)

# Load the dictionary that was used to generate the markers.
# There's different aruco marker dictionaries, this code uses 6x6
#dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

# Initialize the detector parameters using default values
#parameters = cv2.aruco.DetectorParameters_create()

at_detector = Detector(families='tag36h11', nthreads=2, quad_decimate=2.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0)

drone_ip='192.168.0.36'
# initialize the webcam as "camera" object
#camera = cv2.VideoCapture("rtsp://192.168.0.34:8554/unicast")
camera = cv2.VideoCapture('rtspsrc location=rtsp://'+drone_ip+':8554/unicast latency=100 ! queue ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
#print(camera.get(cv2.CAP_PROP_BUFFERSIZE))
#camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
#rec_vid = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (1280,720))

master = mavutil.mavlink_connection(device='udpout:'+drone_ip+':17500', source_system=255)
last_send_ts = time.time()
got_attitude = False

#camMat = np.array([[978.05583, 0, 644.3227087], [0, 980.40099676, 377.5166175], [0, 0, 1]])
#distCoeffs = np.array([0.1542403792, -0.207237266, 0.00062078491848, 0.00089082489, -0.2166006565])
camMat = np.array([[9.7805583154190560e+02, 0, 6.4432270873931213e+02], [0, 9.8040099676993566e+02, 3.7751661754419627e+02], [0, 0, 1]])
distCoeffs = np.array([1.5424037927595446e-01, -2.0723726675535267e-01, 6.2078491848616114e-04, 8.9082489283745796e-04, -2.1660065658513647e-01])
rot_y = np.array([[0,0,1],[0,1,0],[-1,0,0]]) #camera x axis to right, drone x axis to forward

#criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#objp = np.zeros((6*7,3), np.float32)
#objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
#newCamMtx, _ = cv2.getOptimalNewCameraMatrix(camMat, distCoeffs, (1280,720), 1, (1280,720))

#axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

# loop that runs the program forever
# at least until the "q" key is pressed
while True:

    # creates an "img" var that takes in a camera frame
    ret, img = camera.read()
    if ret:
        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #img = cv2.undistort(orig_img, camMat, distCoeffs, None, newCamMtx)

        #tags = []
        tags = at_detector.detect(gray, True, (9.7805583154190560e+02, 9.8040099676993566e+02, 6.4432270873931213e+02, 3.7751661754419627e+02), 0.2)
        #print(tags)

        for tag in tags:
            #imgpts, jac = cv.projectPoints(axis, cv2.Rodrigues(tag.pose_R), tag.pose_t, camMat, distCoeffs)
            cv2.drawFrameAxes(img, camMat, distCoeffs, cv2.Rodrigues(tag.pose_R)[0], tag.pose_t, 0.5)

            heading_r_mtx = np.matmul(rot_y, tag.pose_R)
            heading_r_mtx = heading_r_mtx.transpose()
            r_mtx = tag.pose_R.transpose()
            pos = np.matmul(r_mtx, np.negative(tag.pose_t))
            r_quat = Quaternion(matrix=heading_r_mtx)
            master.mav.att_pos_mocap_send(int(time.time()*1000000), (r_quat.w, r_quat.x, -r_quat.z, r_quat.y), pos[0], -pos[2], pos[1])

        # detect aruco tags within the frame
        #markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(img, dictionary, parameters=parameters)

        # draw box around aruco marker within camera frame
        #img = cv2.aruco.drawDetectedMarkers(img, markerCorners, markerIds)

        # if a tag is found...
        #if markerIds is not None:
        #    now_ts = time.time()
        #    if now_ts - last_send_ts > 0.03:
        #        rvecs , tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 0.2, camMat , distCoeffs)
        #        cv2.drawFrameAxes(img, camMat, distCoeffs, rvecs, tvecs, 0.5)

        #        r_mtx = cv2.Rodrigues(rvecs[0][0])[0]
                #print(rotationMatrixToEulerAngles(r_mtx[0]))
                #print(r_mtx[0])
        #        heading_r_mtx = np.matmul(rot_y, r_mtx)
        #        heading_r_mtx = heading_r_mtx.transpose()
        #        r_mtx = r_mtx.transpose()
        #        pos = np.matmul(r_mtx, np.negative(tvecs[0][0]))
        #        r_quat = Quaternion(matrix=heading_r_mtx)
        #        master.mav.att_pos_mocap_send(int(now_ts*1000000), (r_quat.w, r_quat.x, -r_quat.z, r_quat.y), pos[0], -pos[2], pos[1])
        #        last_send_ts = now_ts
                #print(markerIds[0], pos)

        #        cv2.putText(img, ', '.join(map("{:0.2f}".format, pos)), (1, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 200), 2)
            #else:
            #    rvecs , tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 0.2, camMat , distCoeffs)
            #    cv2.drawFrameAxes(img, camMat, distCoeffs, rvecs, tvecs, 0.5)

            # for every tag in the array of detected tags...
            #for i in range(len(markerIds)):
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

        #ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
        #if ret:
            #corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            # Find the rotation and translation vectors.
            #ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, camMat, distCoeffs)
            #cv2.drawFrameAxes(img, camMat, distCoeffs, rvecs, tvecs, 0.5)

        # Display the resulting frame
        cv2.imshow('image_display', img)

        #rec_vid.write(img)

    # handler to press the "q" key to exit the program
    usr_key = cv2.waitKey(1)
    if  usr_key & 0xff == 113:
        print("bye")
        break
    elif usr_key & 0xff == 114:
        master.mav.command_long_send(0, 0, 246, 0, 1, 0, 0, 0, 0, 0, 0) # reboot
    
    msg = master.recv_msg()
    #msg = None
    if msg is not None:
        msg_type = msg.get_type()
        if msg_type == "STATUSTEXT":
            print ("[", msg.get_srcSystem(),"]", msg.text)
        elif msg_type == "TIMESYNC":
            if msg.tc1 == 0: # ardupilot send a timesync message every 10 seconds
                master.mav.system_time_send(int(time.time() * 1000000), 0) # ardupilot ignore time_boot_ms
        #elif msg_type == "HEARTBEAT":
            #if not got_attitude:
                #master.mav.command_long_send(0, 0, 511, 0, 30, 2000000, 0, 0, 0, 0, 0)
        #elif msg_type == "ATTITUDE":
            #print('heading', msg.yaw*180/math.pi)

# When everything done, release the capture
camera.release()
#rec_vid.release()
cv2.destroyAllWindows()
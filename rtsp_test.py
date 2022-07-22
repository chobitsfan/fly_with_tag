import cv2, os

#os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "flags;no_delay"
print(cv2.getBuildInformation())
#vidCap = cv2.VideoCapture('rtsp://192.168.0.34:8554/unicast', cv2.CAP_FFMPEG)
vidCap = cv2.VideoCapture('rtspsrc location=rtsp://192.168.0.34:8554/unicast latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
#print(vidCap.isOpened())
#print(vidCap.get(cv2.CAP_PROP_BUFFERSIZE))
#vidCap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cv2.namedWindow('image_display', cv2.WINDOW_AUTOSIZE)

while True:
    ret, image = vidCap.read()

    if ret:
        cv2.imshow('image_display', image)
        key = cv2.waitKey(1)
        if key == 113: #q
            break
    else:
        print('nothing')
        break

vidCap.release()

cv2.destroyAllWindows()

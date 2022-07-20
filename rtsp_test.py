import cv2

vidCap = cv2.VideoCapture('rtsp://192.168.0.34:8554/unicast')
cv2.namedWindow('image_display', cv2.WINDOW_AUTOSIZE)

while True:
    ret, image = vidCap.read()

    if ret:
        cv2.imshow('image_display', image)
        key = cv2.waitKey(10)
        if key == 113: #q
            break
    else:
        break

vidCap.release()

cv2.destroyAllWindows()
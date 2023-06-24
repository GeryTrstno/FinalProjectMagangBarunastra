import cv2
import imutils

video = cv2.VideoCapture('2022-10-27-152145.webm')

greenLower = (36, 10, 6)
greenUpper = (120, 255, 255)

redLower = (141, 143, 21)
redUpper = (174, 255, 255)

font = cv2.FONT_HERSHEY_SIMPLEX

while True:
    ret, frame = video.read()
    frame = cv2.resize(frame, (640, 480))
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    greenMask = cv2.inRange(hsv, greenLower, greenUpper)
    greenMask = cv2.erode(greenMask, None, iterations=2)
    greenMask = cv2.dilate(greenMask, None, iterations=2)

    redMask = cv2.inRange(hsv, redLower, redUpper)
    redMask = cv2.erode(redMask, None, iterations=2)
    redMask = cv2.dilate(redMask, None, iterations=2)

    green_contours = cv2.findContours(greenMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours = imutils.grab_contours(green_contours)
    green_centers = []

    for c in green_contours:
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if radius > 10:
            green_centers.append((center, radius))

    for center, radius in green_centers:
        cv2.putText(frame, 'Green Ball', (int(center[0]), int(center[1]) - 30), font, 0.5, (0, 255, 255), 2, cv2.LINE_4)
        cv2.rectangle(frame, (int(center[0] - radius), int(center[1] - radius)), (int(center[0] + radius), int(center[1] + radius)), (0, 255, 0), 2)

    red_contours = cv2.findContours(redMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_contours = imutils.grab_contours(red_contours)
    red_centers = []

    for c in red_contours:
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if radius > 10:
            red_centers.append((center, radius))

    for center, radius in red_centers:
        cv2.putText(frame, 'Red Ball', (int(center[0]), int(center[1]) - 30), font, 0.5, (0, 255, 255), 2, cv2.LINE_4)
        cv2.rectangle(frame, (int(center[0] - radius), int(center[1] - radius)), (int(center[0] + radius), int(center[1] + radius)), (0, 0, 255), 2)

    cv2.imshow("Frame", frame)
    key = cv2.waitKey(10) & 0xFF

    if key == ord("q"):
        break

video.release()
cv2.destroyAllWindows()

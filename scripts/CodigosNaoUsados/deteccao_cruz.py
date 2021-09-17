import cv2
frame = cv2.imread("cross.png")
cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
frame_threshold = cv2.inRange(frame, (0, 240, 240), (20, 255, 255))
contours, hierarchy = cv2.findContours(image=frame_threshold, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)
for cnt in contours:
    area = cv2.contourArea(cnt)
    if area>500:
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.15 * peri, True)

        if len(approx) == 4:
            screen = approx
            cv2.drawContours(frame, [screen], -1, (0,255,0), 3)
            M = cv2.moments(cnt)

            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print("Centro X: ")
            print(cx)
            print("Centro Y:")
            print(cy)
            print("Area Ratio: ")
            areaImagem = frame.shape[0] * frame.shape[1]
            areaRatio = area / areaImagem
            print (areaRatio)
cv2.imshow("HSV", frame)
cv2.waitKey(3)
cv2.imshow("InRange", frame_threshold)
cv2.waitKey(0)

import cv2
import numpy as np

#img = cv2.imread('images/little_cross.jpeg')
img = cv2.imread('images/cross_02.png')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
blur = cv2.medianBlur(hsv, 11)
#cv2.imshow("before", blur)
lower = np.array([30, 125, 0])
upper = np.array([60, 255, 255])

mask = cv2.inRange(blur, lower, upper)
res = cv2.bitwise_and(img, img, mask=mask)

contours, hierarchy = cv2.findContours(
    mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

max_w, max_h, max_x, max_y = 0, 0, 0, 0
for cnt in contours:
    x, y, w, h = cv2.boundingRect(cnt)
    if w*h > max_w*max_h:
        max_w = w
        max_h = h
        max_x = x
        max_y = y
    img = cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 1)

img = cv2.rectangle(img, (max_x, max_y),
                    (max_x+max_w, max_y+max_h), (0, 0, 255), 2)
cv2.circle(img, (int(max_x+max_w/2), int(max_y+max_h/2)),
           1, (0, 0, 255), thickness=3)

# Contour Approximation
# epsilon = 0.01*cv2.arcLength(cnt, True)
# approx = cv2.approxPolyDP(cnt, epsilon, True)

# x, y, w, h = cv2.boundingRect(cnt)
# cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

# rect = cv2.minAreaRect(cnt)
# box = cv2.boxPoints(rect)
# box = np.int0(box)
# cv2.drawContours(img, [box], 0, (0, 0, 255), 2)

# cv2.imshow("Boxes", img)
#cv2.imshow("mask ", mask)
cv2.imshow('stack', np.hstack([img, res]))
cv2.waitKey(0)

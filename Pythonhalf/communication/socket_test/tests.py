import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
print("Success:", ret)
cap.release()
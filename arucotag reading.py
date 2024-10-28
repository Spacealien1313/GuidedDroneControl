import cv2
import numpy as np

# camera id
camera_id = int(input("camid: "))

# camera stream view
window = "cam"
cv2.namedWindow(window)

# aruco dict
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters()
arucoDetector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

# open cam
cap = cv2.VideoCapture(camera_id)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# get frame dimensions
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# find center of te frame
frame_center_x = frame_width // 2
frame_center_y = frame_height // 2

while True:
    try:
        # frame by frame cap
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # detect markers
        corners, ids, _ = arucoDetector.detectMarkers(frame)

        # proccess markers if detected
        if ids is not None:
            for i in range(len(ids)):
                aruco_id = ids[i][0]
                print(f"ArUco Tag ID: {aruco_id}")

                # find center of markers
                corner_points = corners[i]
                tag_center_x = int(np.mean(corner_points[0][:, 0]))
                tag_center_y = int(np.mean(corner_points[0][:, 1]))

                # find distance of pixels to center
                delta_x = tag_center_x - frame_center_x
                delta_y = tag_center_y - frame_center_y
                distance_px = int(np.sqrt(delta_x**2 + delta_y**2))

                # Ccalculate perecent devation
                deviation_x_percent = (delta_x / frame_center_x) * 100
                deviation_y_percent = (delta_y / frame_center_y) * 100

                print(f"Tag Center Offset: X={delta_x}px, Y={delta_y}px")
                print(f"Deviation from Center: X={deviation_x_percent:.2f}%, Y={deviation_y_percent:.2f}%")
                print(f"Distance to Frame Center: {distance_px} px")

                # draw on frame
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.circle(frame, (tag_center_x, tag_center_y), 5, (0, 255, 0), -1)

                # draw line to center
                cv2.line(frame, (frame_center_x, frame_center_y), (tag_center_x, tag_center_y), (0, 255, 255), 2)
                
                # put text along line
                mid_x = (frame_center_x + tag_center_x) // 2
                mid_y = (frame_center_y + tag_center_y) // 2
                cv2.putText(frame, f"{distance_px} px", (mid_x, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # draw circle oin the middle
        cv2.circle(frame, (frame_center_x, frame_center_y), 5, (255, 0, 0), -1)

        # open frame inw indowd
        cv2.imshow(window, frame)

        # q is loop break
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    except KeyboardInterrupt:
        print("Terminating...")
        break

# open cam and close window
cap.release()
cv2.destroyAllWindows()

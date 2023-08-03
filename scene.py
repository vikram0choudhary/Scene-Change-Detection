import cv2

def frame_to_time(frame_number, fps):
    # Calculate time in seconds from frame number and fps
    return frame_number / fps

def scene_change_detector(video_path, threshold=1000):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Error: Unable to open video file.")
        return

    fps = cap.get(cv2.CAP_PROP_FPS)  # Get the video's frames per second

    _, prev_frame = cap.read()
    prev_frame = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    prev_frame = cv2.GaussianBlur(prev_frame, (21, 21), 0)

    frame_number = 0  # Initialize frame number

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_number += 1  # Increment frame number

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_frame = cv2.GaussianBlur(gray_frame, (21, 21), 0)

        frame_diff = cv2.absdiff(prev_frame, gray_frame)
        _, threshold_diff = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(threshold_diff.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        scene_change_detected = False
        for contour in contours:
            if cv2.contourArea(contour) > threshold:
                scene_change_detected = True
                break

        if scene_change_detected:
            time_seconds = frame_to_time(frame_number, fps)
            print(f"ALERT!!!!!!Scene change detected at frame: {frame_number}, time: {time_seconds:.2f} seconds")
            # Here, you can trigger your alert mechanism, e.g., sending an email or notification.

        prev_frame = gray_frame

        # Display the video frame
        cv2.imshow("Scene Change Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Replace 'your_video_file.mp4' with the path to your video file.
    threshold_value = 1500
scene_change_detector('C://Users//HP//Desktop//scenedetection//vid.mp4', threshold=threshold_value)

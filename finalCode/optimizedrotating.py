import cv2
import numpy as np
import threading
import time as t

# Color ranges in HSV
LOWER_RED1 = np.array([0, 100, 100])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([160, 100, 100])
UPPER_RED2 = np.array([180, 255, 255])

LOWER_BLUE = np.array([100, 100, 100])
UPPER_BLUE = np.array([130, 255, 255])

LOWER_GREEN = np.array([40, 100, 100])
UPPER_GREEN = np.array([80, 255, 255])

LOWER_YELLOW = np.array([20, 100, 100])
UPPER_YELLOW = np.array([35, 255, 255])

# Minimum contour area to filter out noise
MIN_CONTOUR_AREA = 500

def rotate_robot(kobuki, duration, stop_event):
    """
    Rotate the robot for a specified duration or until a target color is detected
    
    Args:
        kobuki: Robot control object
        duration (float): Maximum rotation duration in seconds
        stop_event (threading.Event): Event to signal when to stop rotation
    """
    start_time = t.time()
    while t.time() - start_time < duration and not stop_event.is_set():
        kobuki.move(-160, 80, 1)  # Left rotation
        t.sleep(0.1)
    
    # Stop the robot after duration or when stopped by detection
    kobuki.move(0, 0, 1)

def detect_color_boxes(frame, target_color, detected_colors, detected_target, stop_event, horizontal_line_y, exit_event):
    """
    Detect color boxes in the middle third of the frame above the horizontal threshold line
    
    Args:
        frame (numpy.ndarray): Input frame to process
        target_color (str): Target color to look for ('red', 'blue', 'green', 'yellow')
        detected_colors (dict): Dictionary to store detected color information
        detected_target (list): List to store if the target color was detected
        stop_event (threading.Event): Event to signal when to stop rotation
        horizontal_line_y (int): Y-coordinate of horizontal threshold line
        exit_event (threading.Event): Event to signal when to exit the thread
    
    Returns:
        numpy.ndarray: Processed frame with color box detections
        bool: True if target color was detected
    """
    height, width = frame.shape[:2]
    
    # Calculate vertical thirds
    third_width = width // 3
    
    # Define middle section boundaries
    middle_section_start = third_width
    middle_section_end = third_width * 2
    
    # Create a mask for the middle section above the horizontal line
    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    mask[0:horizontal_line_y, middle_section_start:middle_section_end] = 255
    
    # Draw vertical lines to show the middle section
    cv2.line(frame, (middle_section_start, 0), (middle_section_start, height), (255, 255, 255), 2)
    cv2.line(frame, (middle_section_end, 0), (middle_section_end, height), (255, 255, 255), 2)
    
    # Draw horizontal threshold line
    cv2.line(frame, (0, horizontal_line_y), (width, horizontal_line_y), (0, 255, 255), 2)
    cv2.putText(frame, "Detection Zone", (middle_section_start + 10, horizontal_line_y - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    
    # Convert the frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create masks for each color
    # Red mask combines two ranges
    red_mask1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
    red_mask2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    
    blue_mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
    green_mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
    yellow_mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
    
    # Apply the detection zone mask to all color masks
    red_mask = cv2.bitwise_and(red_mask, mask)
    blue_mask = cv2.bitwise_and(blue_mask, mask)
    green_mask = cv2.bitwise_and(green_mask, mask)
    yellow_mask = cv2.bitwise_and(yellow_mask, mask)
    
    # Apply morphological operations to clean up the masks
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
    
    # Find contours for each color
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Color detection colors
    color_map = {
        'red': (0, 0, 255),      # BGR for Red
        'blue': (255, 0, 0),     # BGR for Blue
        'green': (0, 255, 0),    # BGR for Green
        'yellow': (0, 255, 255)  # BGR for Yellow
    }
    
    # Process and draw bounding boxes for each color
    colors_contours = [
        ('red', red_contours),
        ('blue', blue_contours),
        ('green', green_contours),
        ('yellow', yellow_contours)
    ]
    
    target_detected = False
    
    for color_name, contours in colors_contours:
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > MIN_CONTOUR_AREA:
                # Store detected color information
                detected_colors[color_name].append({
                    'contour': contour,
                    'area': area
                })
                
                # Get bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)
                
                # Draw rectangle
                cv2.rectangle(frame, (x, y), (x + w, y + h), color_map[color_name], 2)
                
                # Put text
                cv2.putText(frame, color_name.capitalize(), (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_map[color_name], 2)
                
                # Print detection info
                print(f"{color_name.capitalize()} box detected at ({x}, {y}), size: {w}x{h}")
                
                # Check if this is the target color
                if color_name == target_color and not target_detected:
                    detected_target[0] = True
                    target_detected = True
                    print(f"Target color {target_color} detected! Stopping rotation.")
                    stop_event.set()
                    exit_event.set()  # Signal to exit the detection loop
    
    # Add color detection status on the frame
    offset = 30
    
    # Display detection status for each color
    color_names = ['red', 'blue', 'green', 'yellow']
    for i, color in enumerate(color_names):
        contours = [c for c in colors_contours[i][1] if cv2.contourArea(c) > MIN_CONTOUR_AREA]
        status_text = f"{color.capitalize()}: {'Detected' if contours else 'Not Detected'}"
        
        # Highlight target color
        if color == target_color:
            status_text += " (Target)"
            color_status = color_map[color] if contours else (200, 200, 200)
        else:
            color_status = color_map[color] if contours else (200, 200, 200)
        
        cv2.putText(frame, status_text, (10, (i+1)*offset), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_status, 2)
    
    return frame, target_detected

def detect_colors(duration, target_color, detected_colors, detected_target, stop_event, exit_event, horizontal_line_position=0.6):
    """
    Perform color detection continuously, looking for target color
    
    Args:
        duration (float): Maximum detection duration in seconds
        target_color (str): Target color to look for ('red', 'blue', 'green', 'yellow')
        detected_colors (dict): Dictionary to store detected color information
        detected_target (list): List to store if the target color was detected
        stop_event (threading.Event): Event to signal when to stop rotation
        exit_event (threading.Event): Event to signal when to exit the thread
        horizontal_line_position (float): Position of horizontal threshold line (0.0-1.0, from top)
    """
    # Initialize cap to None
    cap = None
    
    try:
        # Open camera capture
        cap = cv2.VideoCapture(0)
        
        # Get frame dimensions
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture initial frame")
            return
        
        height = frame.shape[0]
        
        # Calculate horizontal line position
        horizontal_line_y = int(height * horizontal_line_position)
        
        start_time = t.time()
        while t.time() - start_time < duration and not exit_event.is_set():
            # Capture frame
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame")
                break
            
            # Detect color boxes
            processed_frame, target_detected = detect_color_boxes(
                frame, target_color, detected_colors, detected_target, stop_event, horizontal_line_y, exit_event
            )
            
            # Display the frame
            cv2.imshow('Color Detection', processed_frame)
            
            # Break if key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # Small delay to control loop rate
            t.sleep(0.1)
    
    finally:
        # Clean up
        if cap is not None:
            cap.release()
        
        # Close any OpenCV windows
        cv2.destroyAllWindows()

def run_detection_rotation(kobuki, duration, target_color, horizontal_line_position=0.6):
    """
    Run color detection and robot rotation in parallel, 
    stopping if target color is detected
    
    Args:
        kobuki: Robot control object
        duration (float): Maximum rotation and detection duration in seconds
        target_color (str): Target color to look for ('red', 'blue', 'green', or 'yellow')
        horizontal_line_position (float): Position of horizontal threshold line (0.0-1.0, from top)
    
    Returns:
        bool: True if target color was detected, False otherwise
    """
    # Initialize detected colors and target
    detected_colors = {
        'red': [],
        'blue': [],
        'green': [],
        'yellow': []
    }
    detected_target = [False]  # Using a list to store the target detection status for thread access
    stop_event = threading.Event()  # Signal to stop rotation
    exit_event = threading.Event()  # Signal to exit the thread
    
    # Create threads for rotation and color detection
    rotation_thread = threading.Thread(target=rotate_robot, args=(kobuki, duration, stop_event))
    detection_thread = threading.Thread(
        target=detect_colors, 
        args=(duration, target_color, detected_colors, detected_target, stop_event, exit_event, horizontal_line_position)
    )
    
    # Set threads as daemon so they don't block program exit
    rotation_thread.daemon = True
    detection_thread.daemon = True
    
    # Start both threads
    rotation_thread.start()
    detection_thread.start()
    
    try:
        # Wait for the rotation thread to complete
        rotation_thread.join(timeout=duration+1)
        
        # Signal the detection thread to exit if it hasn't already
        exit_event.set()
        
        # Give detection thread a short time to clean up and exit
        detection_thread.join(timeout=2.0)
        
    except KeyboardInterrupt:
        # Handle if user presses Ctrl+C
        print("Detection interrupted by user")
        stop_event.set()
        exit_event.set()
    
    finally:
        # Make sure OpenCV windows are closed
        cv2.destroyAllWindows()
        
        # Force stop rotation if still running
        if kobuki is not None:
            kobuki.move(0, 0, 1)
    
    # Return whether target was detected
    return detected_target[0]

def find_color_box(kobuki, target_color, duration=30, horizontal_line_position=0.6):
    """
    Rotate the robot and find specified color box, stopping on detection
    Only detects in the middle third of the frame above the horizontal threshold line
    
    Parameters:
    - kobuki: Robot control object
    - target_color: String color to look for ('red', 'blue', 'green', or 'yellow')
    - duration: Maximum time to rotate and detect (default 30 seconds)
    - horizontal_line_position: Position of horizontal threshold line (0.0-1.0, from top)
                               Default is 0.6 (60% from the top)
    
    Returns:
    - Boolean indicating if the target color was detected
    """
    if target_color not in ['red', 'blue', 'green', 'yellow']:
        print(f"Invalid target color: {target_color}. Must be 'red', 'blue', 'green', or 'yellow'.")
        return False
    
    try:
        target_detected = run_detection_rotation(kobuki, duration, target_color, horizontal_line_position)
        
        if target_detected:
            print(f"Successfully detected a {target_color} box!")
            return True
        else:
            print(f"No {target_color} box detected during rotation.")
            return False
            
    finally:
        # Ensure all OpenCV windows are closed and resources released
        cv2.destroyAllWindows()

# Example usage:
# detected = find_color_box(kobuki, target_color='red', duration=20, horizontal_line_position=0.7)
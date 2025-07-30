import cv2
import numpy as np
import threading
import time as t
from kobukidriversample import Kobuki

def robot_navigation(kobuki, color="white", y_center=240, gap=40, x_center=320, forward_speed=150, timeout=60):
    """
    Robot navigation that:
    1. Detects color boxes ONLY above the upper horizontal line
    2. Selects the closest box to the upper horizontal line as the target
    3. Rotates until color box is between vertical lines
    4. Moves forward until the box crosses below the horizontal lines
    5. Maintains horizontal alignment during forward movement
    6. Stops when complete
    
    Args:
        kobuki: Kobuki robot object
        color (str): Color to detect (default: 'white')
        y_center (int): Y-coordinate of the center point between horizontal lines
        gap (int): Gap between the two horizontal lines
        x_center (int): X-coordinate center of the frame
        forward_speed (int): Speed for forward movement
        timeout (int): Maximum time in seconds before function returns even if not completed
        
    Returns:
        bool: True if navigation successful, False if error occurred or timeout
    """
    # Calculate the y-coordinates for horizontal lines
    line1_y = y_center - gap//2  # Upper horizontal line
    line2_y = y_center + gap//2  # Lower horizontal line
    
    # Calculate x-coordinates for vertical lines (for centering)
    vertical_gap = 80  # Width of the target zone
    line_left_x = x_center - vertical_gap//2
    line_right_x = x_center + vertical_gap//2
    
    # Define color ranges in HSV
    color_ranges = {
        'red': [
            (np.array([0, 100, 100]), np.array([10, 255, 255])),
            (np.array([160, 100, 100]), np.array([180, 255, 255]))
        ],
        'blue': [(np.array([100, 100, 100]), np.array([130, 255, 255]))],
        'green': [(np.array([40, 100, 100]), np.array([80, 255, 255]))],
        'yellow': [(np.array([20, 100, 100]), np.array([35, 255, 255]))],
        'white': [(np.array([0, 0, 200]), np.array([180, 30, 255]))]
    }
    
    # Check if the requested color is supported
    if color not in color_ranges:
        print(f"Error: Color '{color}' not supported.")
        return False
    
    # Create stop flag for threads
    stop_flag = threading.Event()
    
    # Create a timeout flag
    timeout_flag = False
    
    # Start time for timeout calculation
    start_time = t.time()
    
    # Variables for movement control
    is_moving = False
    is_rotating = False
    
    # Flag to ensure box is stable in center position before moving forward
    centered_frames_count = 0
    required_centered_frames = 5  # Number of consecutive frames box must be centered
    
    # Create a movement command queue to communicate between threads
    movement_command = {"left_speed": 0, "right_speed": 0, "movement_type": 0}
    
    cap = None  # Initialize cap outside try block so we can release it in finally
    movement_thread = None  # Initialize thread reference
    
    try:
        # Open video capture
        cap = cv2.VideoCapture(0)
        
        # Check if camera opened successfully
        if not cap.isOpened():
            print("Error: Could not open camera.")
            return False
        
        # Get frame dimensions
        ret, test_frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            return False
            
        height, width = test_frame.shape[:2]
        x_center = width // 2  # Use actual frame center
        
        # Recalculate the vertical line positions
        line_left_x = x_center - vertical_gap//2
        line_right_x = x_center + vertical_gap//2
        
        # Navigation state
        # 0: Rotating to center the box
        # 1: Moving forward until box crosses below lower line
        # 2: Navigation complete
        nav_state = 0
        
        # Variables for direction control
        rotate_direction = 0  # 0: no rotation, -1: left, 1: right
        
        # Variables to track box loss
        box_lost_counter = 0
        max_box_lost_frames = 10  # Maximum frames to allow box loss before stopping
        
        # Track the box we're following
        tracked_box = None  # Will store (x, y, w, h) of the box we're tracking
        
        # Stop robot initially
        stop(kobuki)
        
        # Thread for movement control
        def movement_control():
            nonlocal is_moving, is_rotating
            
            while not stop_flag.is_set():
                # Execute the latest movement command
                try:
                    kobuki.move(
                        movement_command["left_speed"], 
                        movement_command["right_speed"], 
                        movement_command["movement_type"]
                    )
                    
                    # Update movement status flags
                    is_rotating = (movement_command["movement_type"] == 1 and 
                                  (movement_command["left_speed"] != 0 or movement_command["right_speed"] != 0))
                    
                    is_moving = (movement_command["movement_type"] == 0 and 
                                (movement_command["left_speed"] != 0 or movement_command["right_speed"] != 0))
                    
                except Exception as e:
                    print(f"Movement error: {e}")
                    
                # Sleep to avoid hogging CPU and allow robot to execute command
                t.sleep(0.05)
        
        # Start movement control thread
        movement_thread = threading.Thread(target=movement_control)
        movement_thread.daemon = True  # Mark as daemon so it won't block program exit
        movement_thread.start()
        
        # Main navigation loop
        while not stop_flag.is_set():
            # Check for timeout
            if timeout > 0 and (t.time() - start_time) > timeout:
                print(f"Navigation timed out after {timeout} seconds")
                timeout_flag = True
                break
                
            # Capture frame
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                continue
            
            # Get frame dimensions
            height, width = frame.shape[:2]
            
            # Draw the horizontal lines on the frame
            cv2.line(frame, (0, line1_y), (width, line1_y), (255, 0, 0), 2)  # Upper horizontal line (blue)
            cv2.line(frame, (0, line2_y), (width, line2_y), (0, 0, 255), 2)  # Lower horizontal line (red)
            
            # Draw vertical lines for centering
            cv2.line(frame, (line_left_x, 0), (line_left_x, height), (0, 255, 0), 2)  # Left vertical line (green)
            cv2.line(frame, (line_right_x, 0), (line_right_x, height), (0, 255, 0), 2)  # Right vertical line (green)
            
            # Convert frame to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Create a combined mask for the specified color
            mask = None
            for lower, upper in color_ranges[color]:
                current_mask = cv2.inRange(hsv, lower, upper)
                if mask is None:
                    mask = current_mask
                else:
                    mask = cv2.bitwise_or(mask, current_mask)
            
            # Apply morphological operations to clean up the mask
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Default: no box found
            box_found = False
            
            if nav_state == 0:
                # If we're in state 0 (searching), only show detection area marker
                cv2.putText(frame, "DETECTION: FULL FRAME", (10, height - 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                if contours and len(contours) > 0:
                    # Filter contours by minimum size
                    valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]
                    
                    if valid_contours:
                        # Find the closest contour to the upper horizontal line
                        closest_contour = None
                        closest_distance = float('inf')
                        
                        for contour in valid_contours:
                            x, y, w, h = cv2.boundingRect(contour)
                            box_bottom = y + h
                            # Calculate distance to the upper horizontal line
                            distance_to_line = abs(box_bottom - line1_y)
                            
                            # Mark all valid contours with light blue box
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 0), 1)
                            
                            # If this contour is closer to the line than the previous closest
                            if distance_to_line < closest_distance:
                                closest_distance = distance_to_line
                                closest_contour = contour
                        
                        if closest_contour is not None:
                            # Get bounding rectangle of the closest contour
                            x, y, w, h = cv2.boundingRect(closest_contour)
                            box_center_x = x + w//2
                            box_center_y = y + h//2
                            
                            box_found = True
                            tracked_box = (x, y, w, h)
                            box_lost_counter = 0
                            
                            # Draw bounding box for closest contour
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)
                            
                            # Put text with color and "TARGET" label
                            cv2.putText(frame, f"{color.capitalize()} (TARGET)", 
                                      (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                            
                            # NAVIGATION STATE MACHINE
                            # Rotating to center the box
                            if box_center_x < line_left_x:
                                # Box is to the left, rotate left (counter-clockwise)
                                rotate_direction = -1
                                centered_frames_count = 0  # Reset counter
                                
                                # Update movement command for left rotation
                                movement_command["left_speed"] = -160
                                movement_command["right_speed"] = 80
                                movement_command["movement_type"] = 1
                                
                                cv2.putText(frame, "ROTATING LEFT", (width//2 - 100, 50), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                                
                            elif box_center_x > line_right_x:
                                # Box is to the right, rotate right (clockwise)
                                rotate_direction = 1
                                centered_frames_count = 0  # Reset counter
                                
                                # Update movement command for right rotation
                                movement_command["left_speed"] = 80
                                movement_command["right_speed"] = -160
                                movement_command["movement_type"] = 1
                                
                                cv2.putText(frame, "ROTATING RIGHT", (width//2 - 100, 50), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                                
                            else:
                                # Box is centered between vertical lines - STOP ROTATION IMMEDIATELY
                                rotate_direction = 0
                                
                                # Stop rotation
                                movement_command["left_speed"] = 0
                                movement_command["right_speed"] = 0
                                movement_command["movement_type"] = 0
                                
                                cv2.putText(frame, f"BOX CENTERED ({centered_frames_count}/{required_centered_frames})", 
                                         (width//2 - 150, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                                
                                # Increment counter for stable centering
                                centered_frames_count += 1
                                
                                # Only switch to forward movement when box has been centered for enough frames
                                if centered_frames_count >= required_centered_frames:
                                    # Switch to forward movement state
                                    nav_state = 1
                                    
                                    # Start moving forward immediately
                                    movement_command["left_speed"] = forward_speed
                                    movement_command["right_speed"] = forward_speed
                                    movement_command["movement_type"] = 0
                                    
                                    print("Box centered! Switching to forward movement.")
            
            # If in movement state, keep tracking the box we already found
            elif nav_state == 1 and contours and len(contours) > 0 and tracked_box is not None:
                # Get the position of our tracked box
                prev_x, prev_y, prev_w, prev_h = tracked_box
                prev_center_x = prev_x + prev_w//2
                prev_center_y = prev_y + prev_h//2
                
                # Find the contour closest to our tracked box's last position
                closest_contour = None
                closest_distance = float('inf')
                
                for contour in contours:
                    if cv2.contourArea(contour) > 500:  # Filter small contours
                        x, y, w, h = cv2.boundingRect(contour)
                        box_center_x = x + w//2
                        box_center_y = y + h//2
                        
                        # Calculate distance between centers
                        distance = np.sqrt((box_center_x - prev_center_x)**2 + (box_center_y - prev_center_y)**2)
                        
                        if distance < closest_distance:
                            closest_distance = distance
                            closest_contour = contour
                
                # If the closest contour is within a reasonable distance, consider it the same box
                if closest_contour is not None and closest_distance < 150:
                    x, y, w, h = cv2.boundingRect(closest_contour)
                    box_center_x = x + w//2
                    box_center_y = y + h//2
                    
                    box_found = True
                    tracked_box = (x, y, w, h)
                    box_lost_counter = 0
                    
                    # Draw bounding box
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    
                    # Put text with color and "TRACKING" label
                    cv2.putText(frame, f"{color.capitalize()} (TRACKING)", 
                              (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                    # Check if the box has crossed the lower horizontal line
                    if (y + h) > line2_y:
                        # Box has crossed the lower horizontal line - stop and complete
                        movement_command["left_speed"] = 0
                        movement_command["right_speed"] = 0
                        movement_command["movement_type"] = 0
                        
                        cv2.putText(frame, "NAVIGATION COMPLETE!", (width//2 - 150, height//2), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
                        print("ROBOT NAVIGATION SUCCESSFUL")
                        
                        # Set state to complete
                        nav_state = 2
                        
                        # Display complete frame for 2 seconds then break
                        cv2.imshow('Robot Navigation', frame)
                        cv2.waitKey(2000)
                        break  # Exit the main loop
                    else:
                        # Update movement based on box position
                        if box_center_x < line_left_x:
                            # Box is drifting left - adjust with differential movement
                            movement_command["left_speed"] = forward_speed//2
                            movement_command["right_speed"] = forward_speed
                            movement_command["movement_type"] = 0
                            
                            cv2.putText(frame, "ADJUSTING LEFT", (width//2 - 100, 50), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 165, 0), 2)
                        elif box_center_x > line_right_x:
                            # Box is drifting right - adjust with differential movement
                            movement_command["left_speed"] = forward_speed
                            movement_command["right_speed"] = forward_speed//2
                            movement_command["movement_type"] = 0
                            
                            cv2.putText(frame, "ADJUSTING RIGHT", (width//2 - 100, 50), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 165, 0), 2)
                        else:
                            # Box is centered - move straight
                            movement_command["left_speed"] = forward_speed
                            movement_command["right_speed"] = forward_speed
                            movement_command["movement_type"] = 0
                            
                            cv2.putText(frame, "MOVING FORWARD", (width//2 - 100, 50), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            elif nav_state == 2:
                # Navigation complete - keep robot stopped
                movement_command["left_speed"] = 0
                movement_command["right_speed"] = 0
                movement_command["movement_type"] = 0
                
                cv2.putText(frame, "NAVIGATION COMPLETE", (width//2 - 150, height//2), 
                          cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
                
                # Display the final frame briefly
                cv2.imshow('Robot Navigation', frame)
                cv2.waitKey(1000)
                
                # Exit the loop
                break
            
            # If no valid box is found, handle based on current state
            if not box_found:
                box_lost_counter += 1
                cv2.putText(frame, f"BOX LOST ({box_lost_counter}/{max_box_lost_frames})", 
                           (width//2 - 120, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                if box_lost_counter > max_box_lost_frames:
                    if nav_state == 0 and rotate_direction != 0:
                        # Continue last rotation to find the box
                        if rotate_direction == -1:
                            # Continue rotating left
                            movement_command["left_speed"] = -160
                            movement_command["right_speed"] = 80
                            movement_command["movement_type"] = 1
                            
                            cv2.putText(frame, "SEARCHING - ROTATING LEFT", (width//2 - 150, 50), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                        else:
                            # Continue rotating right
                            movement_command["left_speed"] = 80
                            movement_command["right_speed"] = -160
                            movement_command["movement_type"] = 1
                            
                            cv2.putText(frame, "SEARCHING - ROTATING RIGHT", (width//2 - 150, 50), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    elif nav_state == 1:
                        # If box lost during forward movement, stop and go back to rotation mode
                        print("Box lost during forward movement - Stopping and returning to search mode")
                        
                        # Stop first
                        movement_command["left_speed"] = 0
                        movement_command["right_speed"] = 0
                        movement_command["movement_type"] = 0
                        
                        cv2.putText(frame, "BOX LOST - RETURNING TO SEARCH", (width//2 - 150, 50), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                        
                        # Go back to rotation state with a slight rotation to find the box
                        nav_state = 0
                        rotate_direction = -1  # Start with rotating left
                        tracked_box = None  # Reset the tracked box
                        centered_frames_count = 0  # Reset centered frames counter
            
            # Display current state
            state_texts = ["Rotating to center", "Moving forward", "Navigation complete"]
            cv2.putText(frame, f"State: {state_texts[nav_state]}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Display the frame
            cv2.imshow('Robot Navigation', frame)
            
            # Break if key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Navigation manually interrupted by user")
                stop_flag.set()
                break
                
            # Small delay to control frame rate
            t.sleep(0.05)
            
            # Additional timeout check to prevent getting stuck in loops
            if timeout > 0 and (t.time() - start_time) > timeout:
                print(f"Navigation timed out after {timeout} seconds")
                timeout_flag = True
                break
        
    except Exception as e:
        print(f"Error during navigation: {e}")
        return False
    finally:
        # Make sure robot is stopped before exiting
        try:
            kobuki.move(0, 0, 0)  # Ensure robot stops
        except Exception as e:
            print(f"Error stopping robot: {e}")
        
        # Set stop flag to terminate threads
        stop_flag.set()
        
        # Wait for thread to terminate with timeout
        if movement_thread is not None and movement_thread.is_alive():
            movement_thread.join(timeout=2.0)
        
        # Release camera resources
        if cap is not None and cap.isOpened():
            cap.release()
        
        # Close all OpenCV windows
        cv2.destroyAllWindows()
        # Call it multiple times to ensure windows are closed
        for i in range(5):
            cv2.waitKey(1)
            t.sleep(0.1)
            cv2.destroyAllWindows()
    
    # Return navigation success (if not timeout)
    if timeout_flag:
        print("Navigation function exited due to timeout")
        return False
        
    return nav_state == 2

# Helper movement functions
def stop(kobuki):
    # Stop the robot by setting both wheel velocities to 0
    try:
        kobuki.move(0, 0, 0)
    except Exception as e:
        print(f"Error stopping robot: {e}")

# Example usage:
if __name__ == "__main__":
    try:
        # Initialize Kobuki robot
        kobuki = Kobuki()
        
        # Robot navigation parameters
        y_center = 240  # Center position of horizontal lines (vertical)
        gap = 50        # Gap between horizontal lines in pixels
        
        print("Starting robot navigation...")
        print("Press 'q' to quit at any time")
        print("IMPORTANT: Selecting the color box CLOSEST to the upper horizontal line")
        
        result = robot_navigation(
            kobuki=kobuki,
            color="green",  # Try with: "red", "blue", "green", "yellow", "white"
            y_center=y_center, 
            gap=gap,
            forward_speed=150,   # Forward movement speed
            timeout=60           # Maximum 60 seconds before automatic termination
        )
        
        if result:
            print("Navigation mission completed successfully!")
        else:
            print("Navigation mission failed or was interrupted.")
            
        # Add your next function call here - it will run after robot_navigation completes
        print("Moving to next function in the sequence...")
        # next_function_call()
        
    except Exception as e:
        print(f"Error initializing robot: {e}")
    finally:
        # Ensure robot is stopped if program terminates
        try:
            kobuki.move(0, 0, 0)
        except:
            pass
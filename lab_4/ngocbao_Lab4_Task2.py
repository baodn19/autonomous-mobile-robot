import time
import numpy as np
from scipy.optimize import minimize
from robot_systems.robot import HamBot

# --- System Constants ---
LANDMARK_POSITIONS = {
    'yellow': (-1.2, 1.2),
    'blue':    (1.2, 1.2),
    'green':  (-1.2, -1.2),
    'red':   (1.2, -1.2)
}

TARGET_COLORS_RGB = {
    'yellow': (193, 189, 29),
    'red':    (174, 45, 68),
    'green':  (11, 99, 80),
    'blue':   (0, 30, 180)
}

LANDMARK_RADIUS_M = 0.04
CAMERA_WIDTH = 640
CENTER_TOLERANCE_PX = 60
ROTATION_SPEED_RPM = 15

def get_cell_index(x, y):
    """
    Maps continuous (x, y) coordinates in meters to grid cell index 1-16.
    Grid is 4x4, cells are 0.6 m x 0.6 m, centered at origin (0,0).
    """
    # Offset by 1.2 m (half the total 2.4 m width/height) and divide by cell size
    col = int(np.floor((x + 1.2) / 0.6))
    row = int(np.floor((1.2 - y) / 0.6))
    
    # Clamp bounds to 0-3 to handle sensor noise or edge boundaries
    col = max(0, min(3, col))
    row = max(0, min(3, row))
    
    # Calculate 1D index (1 to 16)
    return row * 4 + col + 1

def error_function(pos, measurements):
    """Calculates the squared error between measured and predicted distances."""
    error = 0.0
    for color, d_measured in measurements.items():
        lx, ly = LANDMARK_POSITIONS[color]
        d_predicted = np.sqrt((pos[0] - lx)**2 + (pos[1] - ly)**2)
        error += (d_predicted - d_measured)**2
    return error

def perform_trilateration(measurements):
    """Minimizes the error function to estimate (x, y) position."""
    initial_guess = [0.0, 0.0]
    result = minimize(error_function, initial_guess, args=(measurements,))
    return result.x
    
def gather_measurements(robot):
    """Rotates the robot to find landmarks and records LIDAR distances."""
    measurements = {}
    
    # Begin rotating in place
    robot.set_left_motor_speed(-ROTATION_SPEED_RPM)
    robot.set_right_motor_speed(ROTATION_SPEED_RPM)
    
    for color_name, rgb in TARGET_COLORS_RGB.items():
        robot.camera.set_target_colors([rgb], tolerance=0.15)
        print(f"Scanning for {color_name} landmark...")
        
        found = False
        start_time = time.time()
        
        while not found and (time.time() - start_time < 15):
            landmarks = robot.camera.find_landmarks(min_area=800)
            
            
            if landmarks:
                # Get the largest contour assuming it's the target
                target = max(landmarks, key=lambda l: l.width * l.height)
                print(f"Color (R,G,B): ({target.r}, {target.g}, {target.b})")
                
                # Check if the landmark is centered in the camera frame
                if abs(target.x - (CAMERA_WIDTH / 2)) < CENTER_TOLERANCE_PX:
                    robot.stop_motors()
                    time.sleep(0.5) # Stabilize before reading LIDAR
                    
                    scan = robot.get_range_image()
                    front_dist = scan[180]

                    if front_dist > 0:
                        # Convert millimeters to meters, then add the radius
                        true_distance = (front_dist / 1000.0) + LANDMARK_RADIUS_M
                        measurements[color_name] = true_distance
                        print(f"Recorded {color_name}: {true_distance:.2f} m. Color (R,G,B): ({target.r}, {target.g}, {target.b})")
                        found = True
                    
                    # Resume rotation
                    robot.set_left_motor_speed(-ROTATION_SPEED_RPM)
                    robot.set_right_motor_speed(ROTATION_SPEED_RPM)
                    
            time.sleep(0.05)
            
        robot.camera.clear_target_colors()
        
        # Stop early if we have enough measurements to solve (minimum 3)
        if len(measurements) >= 3:
            break

    robot.stop_motors()
    return measurements

def main():
    robot = HamBot(lidar_enabled=True, camera_enabled=True)
    time.sleep(2) # Allow sensors to warm up

    try:
        measurements = gather_measurements(robot)
        
        if len(measurements) >= 3:
            estimated_pos = perform_trilateration(measurements)
            cell_index = get_cell_index(estimated_pos[0], estimated_pos[1])
            
            print("\n--- Localization Results ---")
            print(f"Estimated Position (x, y): ({estimated_pos[0]:.2f}, {estimated_pos[1]:.2f}) m")
            print(f"Grid Cell Index: {cell_index}")
        else:
            print(f"\nFailed to localize. Only acquired {len(measurements)} measurements. Minimum 3 required.")

    finally:
        robot.disconnect_robot()

if __name__ == "__main__":
    main()

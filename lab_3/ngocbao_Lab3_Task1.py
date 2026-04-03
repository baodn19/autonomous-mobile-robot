import time
import sys
from robot_systems.robot import HamBot

class BugZero:
    def __init__(self, wall_follow_side='left'):
        self.robot = HamBot(lidar_enabled=True, camera_enabled=True)
        self.state = 'SEEK_GOAL'
        self.wall_follow_side = wall_follow_side.lower()
        
        # Yellow cylinder target parameters (R, G, B)
        # Adjust these values based on the physical lighting conditions
        self.robot.camera.set_target_colors((194, 51, 73), tolerance=0.12)
        
        # Distance thresholds (RPLidar returns values in millimeters)
        self.OBSTACLE_THRESHOLD_MM = 400.0  # 0.4 m
        self.WALL_TARGET_DIST_MM = 200.0    # 0.3 m
        self.GOAL_STOP_DIST_MM = 250.0      # 0.25 m

        # Base speeds (RPM)
        self.BASE_SPEED = 30
        self.TURN_SPEED = 20

    def get_lidar_minimum(self, scan, start_angle, end_angle):
        """Extracts the minimum valid distance within a specific angle range."""
        angles = scan[start_angle:end_angle]
        valid_distances = [dist for dist in angles if dist > 0]
        return min(valid_distances) if valid_distances else float('inf')

    def get_distances(self, scan):
        """Returns front, left, and right distances."""
        # Index 180 is front, 90 is left, 270 is right [cite: 30, 33, 36]
        front_dist = self.get_lidar_minimum(scan, 170, 190)
        left_dist = self.get_lidar_minimum(scan, 80, 100)
        right_dist = self.get_lidar_minimum(scan, 260, 280)
        return front_dist, left_dist, right_dist

    def seek_goal(self, landmarks):
        """Drives toward the largest detected yellow landmark."""
        if not landmarks:
            # Spin to search for the goal if not in line of sight
            self.robot.set_left_motor_speed(-self.TURN_SPEED)
            self.robot.set_right_motor_speed(self.TURN_SPEED)
            return

        # Assume the largest yellow object is the goal cylinder
        goal = max(landmarks, key=lambda l: l.width * l.height)
        
        print(f"Goal detected at X={goal.x}. Color (R,G,B): ({goal.r}, {goal.g}, {goal.b})")
        
        # Camera resolution is 640x480. Center X is 320.
        error = 320 - goal.x
        # 1. Reduce proportional gain to prevent over-correction
        kp = 0.01 
        correction = error * kp
        
        # 2. Clamp the correction to a safe maximum (e.g., +/- 10 RPM)
        MAX_CORRECTION = 10.0
        correction = max(min(correction, MAX_CORRECTION), -MAX_CORRECTION)

        # Proportional steering toward the goal
        self.robot.set_left_motor_speed(self.BASE_SPEED - correction)
        self.robot.set_right_motor_speed(self.BASE_SPEED + correction)
        
    def wall_follow(self, front_dist, side_dist):
        """Maintains a set distance from the wall based on the chosen side."""
        # 1. Inner corner detected: turn in place away from the wall
        if front_dist < self.OBSTACLE_THRESHOLD_MM:
            if self.wall_follow_side == 'left':
                self.robot.set_left_motor_speed(self.TURN_SPEED)
                self.robot.set_right_motor_speed(-self.TURN_SPEED)
            else:
                self.robot.set_left_motor_speed(-self.TURN_SPEED)
                self.robot.set_right_motor_speed(self.TURN_SPEED)
            return

        # 2. Outer corner detected (wall discontinued): turn gradually toward the wall
        if side_dist >= 350.0:
            # Bypass PID and use fixed speeds to arc around the corner
            outer_wheel_speed = self.BASE_SPEED
            inner_wheel_speed = 10  # Slow down inner wheel to arc
            
            if self.wall_follow_side == 'left':
                self.robot.set_left_motor_speed(inner_wheel_speed)
                self.robot.set_right_motor_speed(outer_wheel_speed)
            else:
                self.robot.set_left_motor_speed(outer_wheel_speed)
                self.robot.set_right_motor_speed(inner_wheel_speed)
            return

        # 3. Normal wall following: Simple P-controller
        error = self.WALL_TARGET_DIST_MM - side_dist
        kp = 0.09
        correction = error * kp

        if self.wall_follow_side == 'left':
            self.robot.set_left_motor_speed(self.BASE_SPEED + correction)
            self.robot.set_right_motor_speed(self.BASE_SPEED - correction)
        else:
            self.robot.set_left_motor_speed(self.BASE_SPEED - correction)
            self.robot.set_right_motor_speed(self.BASE_SPEED + correction)

    def run(self):
        print(f"Starting Bug 0 Algorithm. Wall Follow Side: {self.wall_follow_side}")
        try:
            while True:
                scan = self.robot.get_range_image()
                if scan == -1:
                    time.sleep(0.05)
                    continue

                front_dist, left_dist, right_dist = self.get_distances(scan)
                landmarks = self.robot.camera.find_landmarks(min_area=500)

                # 1. Check Success Condition
                if landmarks and front_dist <= self.GOAL_STOP_DIST_MM:
                    # Stop within 0.25 m of the goal [cite: 104]
                    goal = max(landmarks, key=lambda l: l.width * l.height)
                    if 200 < goal.x < 440: # Ensure we are actually facing it
                        self.robot.stop_motors()
                        print("Goal Reached successfully.")
                        break

                # 2. State Machine Logic
                if self.state == 'SEEK_GOAL':
                    
                    if front_dist < self.OBSTACLE_THRESHOLD_MM:
                        # Obstacle detected in front, transition to Wall Following [cite: 86]
                        self.state = 'WALL_FOLLOW'
                    else:
                        self.seek_goal(landmarks)
                        

                elif self.state == 'WALL_FOLLOW':
                    # Check if line-of-sight is clear to transition back to Motion to Goal [cite: 87]
                    if landmarks and front_dist > (self.OBSTACLE_THRESHOLD_MM * 1.5):
                        goal = max(landmarks, key=lambda l: l.width * l.height)
                        if 200 < goal.x < 440: # Goal is visible and centered
                            self.state = 'SEEK_GOAL'
                            continue
                    
                    side_dist = left_dist if self.wall_follow_side == 'left' else right_dist
                    self.wall_follow(front_dist, side_dist)
                    print({self.state}, ": ", {front_dist}, ", ", {side_dist})

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\nRun interrupted by user.")
        finally:
            self.robot.disconnect_robot()

if __name__ == "__main__":
    side = 'left'
    if len(sys.argv) > 1 and sys.argv[1].lower() in ['left', 'right']:
        side = sys.argv[1].lower()
    
    controller = BugZero(wall_follow_side=side)
    controller.run()

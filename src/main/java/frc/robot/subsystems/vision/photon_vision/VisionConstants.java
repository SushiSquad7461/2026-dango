package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Constants for the Vision subsystem.
 * 
 * This file contains all the tunable parameters for the vision system.
 * Variables that need to be tuned are in ALL CAPS.
 * 
 * DEPENDENCY VERIFICATION:
 * The current PhotonLib version is v2026.2.2.
 * To update or verify dependencies:
 * 1. Check PhotonVision Installation Guide:
 * https://docs.photonvision.org/en/latest/docs/installation/pvic/index.html
 * 2. In VSCode, open the Command Palette (Ctrl+Shift+P / Cmd+Shift+P).
 * 3. Type "WPILib: Manage Vendor Libraries".
 * 4. Select "Check for updates (online)" to update existing libraries.
 * 5. Or select "Install new libraries (online)" and paste the JSON URL if
 * adding for the first time.
 */
public class VisionConstants {

        // =============================================================================
        // CAMERA SUBSYSTEM CONSTANTS
        // =============================================================================

        /**
         * Name of the camera in the PhotonVision UI.
         * 
         * TO TUNE:
         * 1. Open the PhotonVision web interface (usually
         * http://photonvision.local:5800).
         * 2. Go to the "Cameras" tab.
         * 3. Set the "Camera Name" field to match this string EXACTLY.
         */
        public static final String CAMERA_NAME = "frount_cam"; // TODO: Change this to your actual camera name

        /**
         * Physical position of the camera on the robot relative to the robot center.
         * X: Forward, Y: Left, Z: Up
         * Rotation: Roll, Pitch, Yaw
         * 
         * TO TUNE:
         * Measure the camera's position from the center of the robot's wheelbase (on
         * the floor).
         */
        public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
                        new Translation3d(
                                        Units.inchesToMeters(10.0), // X: Forward distance in meters // TODO: TUNE ME
                                        Units.inchesToMeters(0.0), // Y: Left/Right distance in meters // TODO: TUNE ME
                                        Units.inchesToMeters(20.0) // Z: Height off the floor in meters // TODO: TUNE ME
                        ),
                        new Rotation3d(
                                        0.0, // Roll: Tilt left/right in radians // TODO: TUNE ME
                                        Units.degreesToRadians(-15.0), // Pitch: Tilt up/down in radians // TODO: TUNE
                                                                       // ME
                                        0.0 // Yaw: Turn left/right in radians // TODO: TUNE ME
                        ));

        // =============================================================================
        // POSE ESTIMATION CONSTANTS
        // =============================================================================

        /**
         * Standard deviation for vision measurements.
         * Lower numbers = trust vision more.
         * Higher numbers = trust odometry more.
         * 
         * TO TUNE:
         * Increase these values if the pose jumps around too much when a tag is
         * visible.
         * Decrease these values if the pose corrects too slowly when a tag is visible.
         */
        public static final double VISION_STD_DEV_X_METERS = 0.5; // TODO: TUNE ME
        public static final double VISION_STD_DEV_Y_METERS = 0.5; // TODO: TUNE ME
        public static final double VISION_STD_DEV_THETA_RADIANS = Units.degreesToRadians(10.0); // TODO: TUNE ME

}

package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Pose Estimator using PhotonVision.
 * 
 * This class uses AprilTags to estimate the robot's position on the field.
 * 
 * DOCUMENTATION:
 * Robot Pose Estimator:
 * https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
 */
public class PhotonVisionPoseEstimator {

    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera camera;

    public PhotonVisionPoseEstimator() {
        this.camera = new PhotonCamera(VisionConstants.CAMERA_NAME);

        // Load the AprilTag field layout (e.g., 2024 Crescendo, 2025 Reefscape, 2026
        // Dango)
        AprilTagFieldLayout aprilTagFieldLayout;
        try {
            // Loading default field layout.
            // TODO: Change this to the correct field for the 2026 season if available in
            // WPILib
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        } catch (Exception e) {
            aprilTagFieldLayout = null;
            System.err.println("Failed to load AprilTagFieldLayout!");
            e.printStackTrace();
        }

        if (aprilTagFieldLayout != null) {
            // Create the pose estimator
            // Strategy: MULTI_TAG_PNP_ON_COPROCESSOR is usually best if running on a
            // coprocessor
            // otherwise LOWEST_AMBIGUITY or CLOSEST_TO_CAMERA_HEIGHT
            this.photonPoseEstimator = new PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    VisionConstants.ROBOT_TO_CAMERA);
        } else {
            this.photonPoseEstimator = null;
        }
    }

    /**
     * Updates the pose estimator.
     * 
     * @param prevEstimatedRobotPose The current best guess at the robot's pose
     *                               (from odometry).
     * @return An Optional containing the estimated robot pose if available.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator == null)
            return Optional.empty();

        // Get the list of all unread results since the last call
        // This is important because the camera might run faster than the robot loop
        var results = camera.getAllUnreadResults();

        Optional<EstimatedRobotPose> result = Optional.empty();

        // Feed all results to the pose estimator
        // We only care about the latest valid result
        for (PhotonPipelineResult cameraResult : results) {
            Optional<EstimatedRobotPose> pose = photonPoseEstimator.estimateCoprocMultiTagPose(cameraResult);
            if (pose.isPresent()) {
                result = pose;
            }
        }

        if (result.isPresent()) {
            EstimatedRobotPose pose = result.get();
            SmartDashboard.putNumber("Vision/PoseX", pose.estimatedPose.getX());
            SmartDashboard.putNumber("Vision/PoseY", pose.estimatedPose.getY());
            SmartDashboard.putNumber("Vision/PoseAngle", pose.estimatedPose.getRotation().getAngle());
        }

        return result;
    }
}

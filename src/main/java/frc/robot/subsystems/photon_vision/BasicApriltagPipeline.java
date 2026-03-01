package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Basic AprilTag Pipeline using PhotonVision.
 * 
 * This class wraps the PhotonCamera and provides access to detected targets.
 * 
 * DOCUMENTATION:
 * Getting Target Data:
 * https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html
 */
public class BasicApriltagPipeline {

    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult = new PhotonPipelineResult();

    public BasicApriltagPipeline() {
        // Initialize the camera with the name from VisionConstants
        this.camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
    }

    /**
     * Updates the pipeline and posts data to SmartDashboard.
     * Call this periodically (e.g., in robotPeriodic or a subsystem periodic
     * method).
     */
    public void periodic() {
        // Get the list of all unread results since the last call
        // This is important because the camera might run faster than the robot loop
        var results = camera.getAllUnreadResults();

        // If there are new results, update our cached latest result
        if (!results.isEmpty()) {
            latestResult = results.get(results.size() - 1);
        }

        // Use the latest result (either from this frame or the last one we got)
        PhotonPipelineResult result = latestResult;

        // Check if we have any targets
        boolean hasTargets = result.hasTargets();
        SmartDashboard.putBoolean("Vision/HasTargets", hasTargets);

        if (hasTargets) {
            // Get the best target (highest area/closest)
            PhotonTrackedTarget target = result.getBestTarget();

            // Get target information
            int targetID = target.getFiducialId();
            double yaw = target.getYaw();
            double pitch = target.getPitch();
            double area = target.getArea();
            double skew = target.getSkew();

            // Transform fro camera to target
            // Transform3d bestCameraToTarget = target.getBestCameraToTarget();

            // Post to SmartDashboard
            SmartDashboard.putNumber("Vision/TargetID", targetID);
            SmartDashboard.putNumber("Vision/TargetYaw", yaw);
            SmartDashboard.putNumber("Vision/TargetPitch", pitch);
            SmartDashboard.putNumber("Vision/TargetArea", area);
            SmartDashboard.putNumber("Vision/TargetSkew", skew);

            // Helpful logging
            System.out.println("Target ID: " + targetID + ", Yaw: " + yaw);
        }
    }

    /**
     * Gets the latest result from the camera.
     * 
     * @return PhotonPipelineResult
     */
    public PhotonPipelineResult getLatestResult() {
        return latestResult;
    }
}

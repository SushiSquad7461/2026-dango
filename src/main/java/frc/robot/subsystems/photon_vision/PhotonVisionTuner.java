package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Tuner for PhotonVision camera settings.
 * 
 * This class allows for runtime adjustment of camera settings (exposure,
 * brightness, etc.)
 * primarily through SmartDashboard or NetworkTables.
 * 
 * DOCUMENTATION:
 * Tuning Color Pipelines:
 * https://docs.photonvision.org/en/latest/docs/tuning/index.html
 */
public class PhotonVisionTuner {

    private final PhotonCamera camera;

    // Default tuning values - CHANGE THESE TO TUNE
    private boolean driverMode = false; // TODO: TUNE ME
    // Note: Most driver camera property tuning (exposure, gain) happens in the
    // PhotonVision UI,
    // not through code API for standard AprilTag pipelines.
    // However, you can switch pipelines or toggle driver mode.

    public PhotonVisionTuner() {
        this.camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
        initDashboard();
    }

    private void initDashboard() {
        SmartDashboard.putBoolean("Vision/Tuning/DriverMode", driverMode);
        SmartDashboard.putNumber("Vision/Tuning/PipelineIndex", 0);
    }

    /**
     * Periodic method to check for tuning updates from SmartDashboard.
     */
    public void periodic() {
        // Driver Mode
        boolean newDriverMode = SmartDashboard.getBoolean("Vision/Tuning/DriverMode", driverMode);
        if (newDriverMode != driverMode) {
            driverMode = newDriverMode;
            camera.setDriverMode(driverMode);
            System.out.println("Vision: Driver Mode set to " + driverMode);
        }

        // Pipeline Switching
        int pipelineIndex = (int) SmartDashboard.getNumber("Vision/Tuning/PipelineIndex", 0);
        // Look for a change (implement your own logic to track previous index if
        // needed,
        // or just set it periodically if inexpensive)
        camera.setPipelineIndex(pipelineIndex);
    }
}

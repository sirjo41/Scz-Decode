package org.firstinspires.ftc.teamcode.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.SpindexerAuto;
import org.firstinspires.ftc.teamcode.spindexer.Spindexer;

import java.util.List;

/**
 * Wrapper class for Limelight 3A camera.
 * Handles initialization, pipeline switching, and AprilTag result parsing for
 * GamePattern selection.
 */
public class LimelightControl {

    private Limelight3A limelight;
    private Telemetry telemetry;

    public LimelightControl(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0); // Default to pipeline 0 (AprilTags)
            limelight.start();
        } catch (Exception e) {
            telemetry.addData("Limelight Error", "Could not initialize: " + e.getMessage());
        }
    }

    /**
     * Set the running pipeline.
     * 
     * @param index Pipeline index (0-9)
     */
    public void setPipeline(int index) {
        if (limelight != null) {
            limelight.pipelineSwitch(index);
        }
    }

    /**
     * Get the latest result from the Limelight.
     * 
     * @return LLResult object or null if not available
     */
    public LLResult getLatestResult() {
        if (limelight != null) {
            return limelight.getLatestResult();
        }
        return null;
    }

    /**
     * Determines the Spindexer.GamePattern based on the primary detected AprilTag.
     * Mapping:
     * - ID 21 -> GREEN_FIRST
     * - ID 22 -> GREEN_SECOND
     * - ID 23 -> GREEN_THIRD
     *
     * @return Resolved GamePattern or null if no valid tag found
     */
    public SpindexerAuto.GamePattern getGamePatternFromTags() {
        LLResult result = getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }

        // Get fiducial results (AprilTags)
        List<LLResultTypes.FiducialResult> fidutials = result.getFiducialResults();
        if (fidutials.isEmpty()) {
            return null;
        }

        // Look at the first detected tag (usually the closest or most central depending
        // on sorting)
        int id = fidutials.get(0).getFiducialId();

        switch (id) {
            case 21:
                return Spindexer.GamePattern.GREEN_FIRST;
            case 22:
                return Spindexer.GamePattern.GREEN_SECOND;
            case 23:
                return Spindexer.GamePattern.GREEN_THIRD;
            default:
                return null;
        }
    }

    /**
     * Gets the robot's pose from Limelight MegaTag.
     * Converts from Limelight (Meters) to PedroPathing (Inches).
     * 
     * @return Pose in inches and radians, or null if invalid.
     */
    public Pose getRobotPose() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                // Convert Meters to Inches
                double x_inches = botpose.getPosition().x * 39.3701;
                double y_inches = botpose.getPosition().y * 39.3701;

                // Yaw is usually in Degrees or Radians depending on API.
                double heading_radians = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

                return new Pose(x_inches, y_inches, heading_radians);
            }
        }
        return null;
    }

    /**
     * Stops the Limelight camera.
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    public void telemetry() {
        if (limelight == null) {
            telemetry.addLine("Limelight: NOT FOUND");
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            telemetry.addData("Tx", result.getTx());
            telemetry.addData("Ty", result.getTy());
            telemetry.addData("Ta", result.getTa());

            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            if (!tags.isEmpty()) {
                telemetry.addData("Tag ID", tags.get(0).getFiducialId());
            } else {
                telemetry.addData("Tag ID", "None");
            }
        } else {
            telemetry.addLine("No Valid Data");
        }
    }
}

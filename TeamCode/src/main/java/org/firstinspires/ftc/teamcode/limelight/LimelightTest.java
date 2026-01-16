package org.firstinspires.ftc.teamcode.limelight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.spindexer.Spindexer;

/**
 * TeleOp test for Limelight integration.
 * Displays camera stream and detected AprilTags.
 * Shows which GamePattern would be selected based on the tag.
 */
@TeleOp(name = "LimelightTest", group = "Tests")
public class LimelightTest extends LinearOpMode {

    private LimelightControl limelight;

    @Override
    public void runOpMode() {
        // Initialize Limelight wrapper
        limelight = new LimelightControl(hardwareMap, telemetry);

        telemetry.addLine("Limelight Test Initialized");
        telemetry.addLine("Show AprilTagsgit push origin <your-branch-name> --force\n (11, 12, 13)");
        telemetry.addLine("to see resolved GamePattern.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get resolved game pattern from detected tags
            Spindexer.GamePattern pattern = limelight.getGamePatternFromTags();

            telemetry.addLine("=== Limelight Pattern Detection ===");
            if (pattern != null) {
                telemetry.addData("Detected Pattern", pattern);
            } else {
                telemetry.addData("Detected Pattern", "NONE / NO TAG");
            }
            telemetry.addLine();

            // Dump raw Limelight status (Tx, Ty, Tag ID)
            limelight.telemetry();
            
            telemetry.update();
        }

        limelight.stop();
    }
}

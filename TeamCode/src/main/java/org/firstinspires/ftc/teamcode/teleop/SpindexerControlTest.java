package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.spindexer.Spindexer;

/**
 * TeleOp test for the Spindexer system with pattern-based ejection.
 * Provides manual control for intake and automatic preparation for ejection.
 */
@TeleOp(name = "SpindexerControlTest", group = "Examples")
public class SpindexerControlTest extends LinearOpMode {

    // Hardware configuration names
    private static final String SPINDEXER_NAME = "spindexer";
    private static final String INTAKE_COLOR_NAME = "intakeColor";
    private static final String INTAKE_NAME = "intake";
    private static final String SHOOTER_NAME = "shooter";
    private static final String FEEDER_NAME = "feeder";
    private static final String SHOOTER_COLOR_NAME = "shooterColor";

    private Spindexer spindexer;

    // Button state tracking for edge detection
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastB = false;
    private boolean lastBack = false;

    // Operational state tracking
    private int loadedCount = 0;
    private boolean preparedForEject = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, SPINDEXER_NAME);
        DcMotor intake = hardwareMap.get(DcMotor.class, INTAKE_NAME);
        ColorSensor intakeColor = hardwareMap.get(ColorSensor.class, INTAKE_COLOR_NAME);
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        Servo feeder = hardwareMap.get(Servo.class, FEEDER_NAME);
        ColorSensor shooterColor = hardwareMap.get(ColorSensor.class, SHOOTER_COLOR_NAME);

        spindexer = new Spindexer(motor, intakeColor, shooter, feeder, shooterColor);

        // Display control instructions
        telemetry.addLine("=== Spindexer Control Test ===");
        telemetry.addLine();
        telemetry.addLine("X: Read color, rotate +120°");
        telemetry.addLine("Y: Eject ALL (follow pattern)");
        telemetry.addLine("B: Cycle ejection pattern");
        telemetry.addLine("A: Intake one ball");
        telemetry.addLine("Back: Re-zero (set current position as slot 0)");
        telemetry.addLine();
        telemetry.addData("Ejection Pattern", spindexer.getGamePattern());
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Edge detection for button presses
            boolean xEdge = gamepad1.x && !lastX;
            boolean yEdge = gamepad1.y && !lastY;
            boolean bEdge = gamepad1.b && !lastB;
            boolean backEdge = gamepad1.back && !lastBack;

            lastX = gamepad1.x;
            lastY = gamepad1.y;
            lastB = gamepad1.b;
            lastBack = gamepad1.back;

            // Intake one ball
            if (xEdge) {
                spindexer.intakeOne(telemetry);
                loadedCount = Math.min(3, loadedCount + 1);
                preparedForEject = false;  // No longer at eject position after intake
            }

            // Auto-preparation: When 3 balls are loaded, align for first eject
            if (!preparedForEject && spindexer.hasThreeBalls()) {
                boolean success = spindexer.prepareFirstEjectByPattern(telemetry);
                preparedForEject = success;
            }

            // Manual eject all (follows game pattern)
            if (yEdge) {
                spindexer.ejectAllByPattern(telemetry);
                loadedCount = 0;
                preparedForEject = false;  // After ejection, returned to intake stop
            }

            // Cycle ejection pattern
            if (bEdge) {
                Spindexer.GamePattern current = spindexer.getGamePattern();
                Spindexer.GamePattern[] patterns = Spindexer.GamePattern.values();
                int nextIdx = (current.ordinal() + 1) % patterns.length;
                spindexer.setGamePattern(patterns[nextIdx]);
                preparedForEject = false; // Re-align for new pattern if needed
            }

            // Re-zero: Define current position as slot 0 at intake
            if (backEdge) {
                spindexer.rezeroHere(); 
                loadedCount = 0;
                preparedForEject = false;
            }
            if (gamepad1.a) {
                intake.setPower(1);
            }
            else {
                intake.setPower(0);
            }

            // Display current state
            Spindexer.Ball[] slots = spindexer.getSlots();

            telemetry.addLine("=== Current State ===");
            telemetry.addLine("=== Current State ===");
            telemetry.addData("Encoder Position", spindexer.getEncoder());
            telemetry.addData("Target Position", spindexer.getTarget());
            telemetry.addData("Shooter Velocity", spindexer.getShooterVelocity());
            telemetry.addData("Target Position", spindexer.getTarget());
            telemetry.addData("Intake Slot Index", spindexer.getIntakeIndex());
            telemetry.addData("Loaded Count", loadedCount);
            telemetry.addLine();
            telemetry.addLine("--- Configuration ---");
            telemetry.addData("Ejection Pattern", spindexer.getGamePattern());
            telemetry.addData("At Eject Position", spindexer.isAtMid());
            telemetry.addData("Prepared for Eject", preparedForEject);
            telemetry.addLine();
            telemetry.addLine("--- Slot Contents ---");
            telemetry.addData("Slot 0", slots[0]);
            telemetry.addData("Slot 1", slots[1]);
            telemetry.addData("Slot 2", slots[2]);
            telemetry.update();
        }
    }
}
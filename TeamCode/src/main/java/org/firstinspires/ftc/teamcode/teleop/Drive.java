package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.limelight.LimelightControl;
import org.firstinspires.ftc.teamcode.spindexer.Spindexer;

@TeleOp(name = "Drive", group = "TeleOp")
public class Drive extends LinearOpMode {

    // Hardware Names
    private static final String MOTOR_FL = "frontLeft";
    private static final String MOTOR_BL = "backLeft";
    private static final String MOTOR_FR = "frontRight";
    private static final String MOTOR_BR = "backRight";

    // Spindexer & Intake Hardware
    private static final String SPINDEXER_MOTOR = "spindexer";
    private static final String COLOR_SENSOR = "shooterColor";
    private static final String INTAKE_MOTOR = "intake";
    private static final String FEEDER_SERVO = "feeder";
    private static final String SHOOTER_MOTOR = "shooter";

    // Subsystems
    private Spindexer spindexer;
    private LimelightControl limelight;
    private DcMotor intake;

    // Edge Detection
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastB = false;
    private boolean lastDpUp = false;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Hardware Initialization ---

        // Drive Motors
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, MOTOR_FL);
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, MOTOR_BL);
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, MOTOR_FR);
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, MOTOR_BR);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Spindexer System
        DcMotorEx spinMotor = hardwareMap.get(DcMotorEx.class, SPINDEXER_MOTOR);
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, COLOR_SENSOR);
        Servo feederServo = hardwareMap.get(Servo.class, FEEDER_SERVO);
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR);
        spindexer = new Spindexer(this, spinMotor, colorSensor, feederServo, shooterMotor);

        // Intake Motor
        intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);

        // Limelight
        limelight = new LimelightControl(hardwareMap, telemetry);

        telemetry.addLine("Initialized! Ready to Start.");
        telemetry.update();
        waitForStart();
        if (isStopRequested())
            return;

        while (opModeIsActive()) {

            // === 1. Drive Control (Gamepad 1) ===
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // === 2. Spindexer Control (Gamepad 1) ===

            // Edge Detection
            boolean leftEdge = gamepad1.dpad_left && !lastX;
            boolean rightEdge = gamepad1.dpad_right && !lastY;
            boolean dpUpEdge = gamepad1.dpad_up && !lastDpUp; // Cycle Pattern

            lastX = gamepad1.dpad_left;
            lastY = gamepad1.dpad_right;
            lastDpUp = gamepad1.dpad_up;

            // A: Intake (Run Intake)
            if (gamepad1.a) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }

            // Update intake system (auto-detect balls and track slots)
            spindexer.updateIntake();

            // BACK Button: Emergency Stop Spindexer
            if (gamepad1.back) {
                spindexer.emergencyStop();
            }

            // DPad Left: Move spindexer left
            if (leftEdge) {
                spindexer.moveLeft(telemetry);
            }

            // DPad Right: Move spindexer right
            if (rightEdge) {
                spindexer.moveRight(telemetry);
            }

            // Update Pattern from Limelight (Auto-Detect)
            Spindexer.GamePattern detected = limelight.getGamePatternFromTags();
            if (detected != null) {
                spindexer.setGamePattern(detected);
            }

            // DPad Up: Manual Pattern Cycle Override
            if (dpUpEdge) {
                Spindexer.GamePattern current = spindexer.getGamePattern();
                Spindexer.GamePattern[] patterns = Spindexer.GamePattern.values();
                int nextIdx = (current.ordinal() + 1) % patterns.length;
                spindexer.setGamePattern(patterns[nextIdx]);
            }

            // === 3. Shooter Control (Gamepad 1) ===

            boolean rightBumperEdge = gamepad1.right_bumper && !lastRightBumper;
            lastRightBumper = gamepad1.right_bumper;

            // Right Trigger: Spin up shooter
            if (gamepad1.right_trigger > 0.1) {
                spindexer.spinUpShooter();
            } else {
                // If not spinning up, allow natural slowdown
                // Don't actively stop to maintain momentum
            }

            // Right Bumper: Shoot (feed ball when ready)
            if (rightBumperEdge) {
                boolean fed = spindexer.shoot();
                if (!fed) {
                    // Could add haptic feedback or telemetry warning
                }
            } else if (!gamepad1.right_bumper) {
                // Retract feeder when button released
                spindexer.retractFeeder();
            }

            // B Button: Kill switch - emergency stop shooter
            if (gamepad1.b) {
                spindexer.stopShooter();
            }

            // === 4. Telemetry ===
            telemetry.addLine("--- Spindexer State ---");
            telemetry.addData("Pattern", spindexer.getGamePattern());
            telemetry.addData("Detected Tag Pattern", detected);
            telemetry.addData("Encoder", spindexer.getEncoder());
            telemetry.addData("Target", spindexer.getTarget());

            // Add slot telemetry
            spindexer.addSlotTelemetry(telemetry);

            // Add shooter telemetry
            spindexer.addShooterTelemetry(telemetry);

            telemetry.addLine("\n--- Controls ---");
            telemetry.addLine("A: Run Intake");
            telemetry.addLine("DPad Left/Right: Move Spindexer");
            telemetry.addLine("DPad Up: Cycle Pattern");
            telemetry.addLine("Right Trigger: Spin Up Shooter");
            telemetry.addLine("Right Bumper: Shoot");
            telemetry.addLine("B: KILL SHOOTER");
            telemetry.addLine("BACK: Emergency Stop Spindexer");

            telemetry.update();
        }

        limelight.stop();
    }
}

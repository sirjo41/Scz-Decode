package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.spindexer.Shooter;
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
    private static final String COLOR_SENSOR = "intakeColor";
    private static final String INTAKE_MOTOR = "intake";
    private static final String FEEDER_SERVO = "feeder";
    private static final String SHOOTER_MOTOR = "shooter";

    // Edge Detection
    private boolean lastDpUp = false;
    private boolean lastDpDown = false;

    @Override
    public void runOpMode() {

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
        NormalizedColorSensor intakeSensor = hardwareMap.get(NormalizedColorSensor.class, COLOR_SENSOR);
        Servo feederServo = hardwareMap.get(Servo.class, FEEDER_SERVO);
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Subsystems
        Shooter shooter = new Shooter(shooterMotor, feederServo);
        Spindexer spindexer = new Spindexer(this, spinMotor, intakeSensor, shooter);

        // Intake Motor
        DcMotor intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
        intakeSensor.setGain(2);

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
            boolean dpUpEdge = gamepad1.dpad_up && !lastDpUp; // Cycle Pattern
            boolean dpDownEdge = gamepad1.dpad_down && !lastDpDown; // Switch Mode

            lastDpUp = gamepad1.dpad_up;
            lastDpDown = gamepad1.dpad_down;

            // A: Intake (Run Intake)
            if (gamepad1.a) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }

            // Update intake system (auto-detect balls and track slots)
            spindexer.updateIntake();

            // Update shooter to maintain velocity
            spindexer.updateShooter();

            // BACK Button: Emergency Stop Spindexer
            if (gamepad1.back) {
                spindexer.emergencyStop();
            }

            // DPad Down: Switch Mode (Intaking <-> Shooting) and move half-slot
            if (dpDownEdge) {
                if (spindexer.getMode() == Spindexer.SpindexerMode.INTAKING) {
                    spindexer.setMode(Spindexer.SpindexerMode.SHOOTING);
                    spindexer.moveRightHalf(); // Move half-slot when switching to shooting
                } else {
                    spindexer.setMode(Spindexer.SpindexerMode.INTAKING);
                    spindexer.moveRightHalf(); // Move half-slot when switching to intaking
                }
            }

            // DPad Left: Move spindexer left (full-slot always)
            if (gamepad1.dpad_left) {
                spindexer.moveLeft();
            }

            // DPad Right: Move spindexer right (full-slot always)
            if (gamepad1.dpad_right) {
                spindexer.moveRight();
            }

            // DPad Up: Manual Pattern Cycle Override
            if (dpUpEdge) {
                Spindexer.GamePattern current = spindexer.getGamePattern();
                Spindexer.GamePattern[] patterns = Spindexer.GamePattern.values();
                int nextIdx = (current.ordinal() + 1) % patterns.length;
                spindexer.setGamePattern(patterns[nextIdx]);
            }

            // === 3. Shooter Control (Gamepad 1) ===

            // Right Trigger: Spin up shooter
            if (gamepad1.right_trigger > 0.1) {
                spindexer.spinUpShooter();
            } else if (gamepad1.b) {
                spindexer.stopShooter();
            }

            if (gamepad1.y) {
                // Manual trigger for shooting (Smart Sort)
                spindexer.shoot();
            }

            // === 4. Telemetry ===
            telemetry.addLine("--- Spindexer State ---");
            telemetry.addData("Mode", spindexer.getMode());
            telemetry.addData("Pattern", spindexer.getGamePattern());
            telemetry.addData("Encoder", spindexer.getEncoder());
            telemetry.addData("Target", spindexer.getTarget());

            // Add slot telemetry
            spindexer.addSlotTelemetry(telemetry);

            // Add shooter telemetry
            spindexer.addShooterTelemetry(telemetry);

            spindexer.addSensorTelemetry(telemetry);

            telemetry.addLine("\n--- Controls ---");
            telemetry.addLine("A: Run Intake");
            telemetry.addLine("DPad Left/Right: Move Spindexer");
            telemetry.addLine("DPad Up: Cycle Pattern");
            telemetry.addLine("DPad Down: Switch Mode (Intaking/Shooting)");
            telemetry.addLine("Right Trigger: Spin Up Shooter");
            telemetry.addLine("Right Bumper: Shoot");
            telemetry.addLine("B: KILL SHOOTER");
            telemetry.addLine("BACK: Emergency Stop Spindexer");

            telemetry.update();
        }
    }
}

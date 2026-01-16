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
    private static final String INTAKE_COLOR = "intakeColor";
    private static final String SHOOTER_MOTOR = "shooter";
    private static final String FEEDER_SERVO = "feeder";
    private static final String SHOOTER_COLOR = "shooterColor";
    private static final String INTAKE_MOTOR = "intake";

    // Subsystems
    private Spindexer spindexer;
    private LimelightControl limelight;
    private DcMotor intake;

    // Edge Detection
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastB = false;
    private boolean lastDpUp = false;

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
        ColorSensor inColor = hardwareMap.get(ColorSensor.class, INTAKE_COLOR);
        DcMotorEx shootMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR);
        Servo feedServo = hardwareMap.get(Servo.class, FEEDER_SERVO);
        ColorSensor shootColor = hardwareMap.get(ColorSensor.class, SHOOTER_COLOR);
        
        spindexer = new Spindexer(spinMotor, inColor, shootMotor, feedServo, shootColor);

        // Intake Motor
        intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);

        // Limelight
        limelight = new LimelightControl(hardwareMap, telemetry);

        telemetry.addLine("Initialized! Ready to Start.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

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
            boolean xEdge = gamepad1.x && !lastX; // Index One / Manual Sort
            boolean yEdge = gamepad1.y && !lastY; // Auto Shoot (Pattern)
            boolean bEdge = gamepad1.b && !lastB; // Manual Eject Single
            boolean dpUpEdge = gamepad1.dpad_up && !lastDpUp; // Cycle Pattern Manual

            lastX = gamepad1.x;
            lastY = gamepad1.y;
            lastB = gamepad1.b;
            lastDpUp = gamepad1.dpad_up;

            // A: Intake (Run Intake)
            if (gamepad1.a) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }
            
            // X: Index One Ball (Manual Sort / Store)
            if (xEdge) {
                // Reads color at intake, stores it, moves 120 degrees
                spindexer.intakeOne(telemetry);
            }

            // Y: Auto Sort / Shoot (Execute Pattern)
            if (yEdge) {
                // This will start shooter, align balls, check colors, and fire
                spindexer.ejectAllByPattern(telemetry);
            }
            
            // B: Manual Eject (Current Slot)
            // Note: Spindexer doesn't have a public ejectCurrentSlot method that actuates servo
            // We can add one or just use internal logic logic if needed, 
            // but for now, let's assume 'ejectAllByPattern' is the primary way.
            // If user wants manual single eject, we might need to expose ejectSlot logic publicly 
            // or just rely on Auto Sort. Let's leave B as a "Cycle Pattern" backup or similar if needed.
            // Actually, let's map B to cycle pattern too, just in case
            
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

            // === 3. Telemetry ===
            telemetry.addLine("--- Spindexer State ---");
            telemetry.addData("Pattern", spindexer.getGamePattern());
            telemetry.addData("Detected Tag Pattern", detected);
            telemetry.addData("Shooter Vel", spindexer.getShooterVelocity());
            
            Spindexer.Ball[] slots = spindexer.getSlots();
            telemetry.addData("Slots", "%s | %s | %s", slots[0], slots[1], slots[2]);
            
            telemetry.update();
        }
        
        limelight.stop();
    }
}

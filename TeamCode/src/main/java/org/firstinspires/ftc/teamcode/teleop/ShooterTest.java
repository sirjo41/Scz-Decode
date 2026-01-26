package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "PedroShooterJoystickTest", group = "Testing")
public class ShooterTest extends LinearOpMode {

    private Shooter shooter;

    private DcMotorEx shooterMotor;
    private Servo feederServo;
    private Servo angleServo;

    private Follower follower;
    private Pose targetPose;

    private static final double PEDRO_TO_METERS = 0.0254; // 1 Pedro unit = 1 inch

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware initialization
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        feederServo = hardwareMap.get(Servo.class, "feederServo");
        angleServo = hardwareMap.get(Servo.class, "angleServo");

        shooter = new Shooter(shooterMotor, feederServo, angleServo);

        follower = Constants.createFollower(hardwareMap);

        // Starting Pose
        Pose startingPose = new Pose(22.355, 123.514, Math.toRadians(143));
        follower.setPose(startingPose);

        // Target Pose
        targetPose = new Pose(16, 131, Math.toRadians(143));

        telemetry.addLine("Initialization complete. Ready to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // =========================
            // JOYSTICK DRIVE
            // =========================
            double driveY = -gamepad1.left_stick_y;
            double driveX = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            follower.setTeleOpDrive(driveY, driveX, turn, true); // Robot-centric

            // =========================
            // CONTROLS
            // =========================
            // Press A → rotate to target heading only
            if (gamepad1.a) {
                Pose currentPose = follower.getPose();
                Pose newPose = new Pose(currentPose.getX(), currentPose.getY(), targetPose.getHeading());
                follower.setPose(newPose);
                telemetry.addLine("Heading aligned to target!");
            }

            if (gamepad1.b) shooter.startShooter();

            if (gamepad1.x) {
                if (shooter.isShooterReady()) {
                    shooter.feed();
                    sleep(500);
                    shooter.retractFeeder();
                }
            }

            if (gamepad1.y) shooter.stopShooter();

            // Update shooter
            shooter.updateShootingSolution(follower, targetPose, PEDRO_TO_METERS);

            // =========================
            // TELEMETRY
            // =========================
            Pose pose = follower.getPose();
            telemetry.addLine("=== Robot Pose ===");
            telemetry.addData("X (Pedro)", pose.getX());
            telemetry.addData("Y (Pedro)", pose.getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));

            shooter.addShooterTelemetry(telemetry);

            telemetry.update();
        }
    }

    // =========================
    // Shooter class
    // =========================
    public static class Shooter {

        private static final double RPM_PER_METER_PER_SECOND = 180;
        private static final double SERVO_MIN_POS = 0.0;
        private static final double SERVO_MAX_POS = 1.0;
        private static final double ANGLE_AT_MIN = 20.0;
        private static final double ANGLE_AT_MAX = 60.0;
        private static final double GRAVITY = 9.81;
        private static final double GOAL_HEIGHT_METERS = 0.9;
        private static final double SHOOTER_HEIGHT_METERS = 0.35;

        private final DcMotorEx shooterMotor;
        private final Servo feederServo;
        private final Servo angleServo;

        public double currentTargetRPM = 0;
        public double currentTargetAngle = 0;
        private boolean shooterActive = false;

        public static double SHOOTER_P = 3;
        public static double SHOOTER_F = 12;

        private static final double FEEDER_IDLE = 1.0;
        private static final double FEEDER_FEEDING = 0.55;

        public Shooter(DcMotorEx shooterMotor, Servo feederServo, Servo angleServo) {
            this.shooterMotor = shooterMotor;
            this.feederServo = feederServo;
            this.angleServo = angleServo;

            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

            shooterMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F));

            feederServo.setPosition(FEEDER_IDLE);
        }

        public void updateShootingSolution(Follower follower, Pose targetPose, double pedroToMeters) {
            if (!shooterActive) {
                shooterMotor.setVelocity(0);
                return;
            }

            Pose robotPose = follower.getPose();
            Vector robotVel = follower.getVelocity();

            double dx = (targetPose.getX() - robotPose.getX()) * pedroToMeters;
            double dy = (targetPose.getY() - robotPose.getY()) * pedroToMeters;
            double distanceToGoal = Math.hypot(dx, dy);

            double desiredAngleDeg = 60 - (distanceToGoal * 0.5);
            double desiredAngleRad = Math.toRadians(desiredAngleDeg);

            double heightDiff = GOAL_HEIGHT_METERS - SHOOTER_HEIGHT_METERS;
            double term1 = GRAVITY * Math.pow(distanceToGoal, 2);
            double term2 = 2 * Math.pow(Math.cos(desiredAngleRad), 2) * (distanceToGoal * Math.tan(desiredAngleRad) - heightDiff);
            double physicsVelocity = Math.sqrt(Math.abs(term1 / term2));

            double angleToGoal = Math.atan2(dy, dx);
            double robotSpeedTowardsGoal = (robotVel.getXComponent() * pedroToMeters * Math.cos(angleToGoal)) +
                    (robotVel.getYComponent() * pedroToMeters * Math.sin(angleToGoal));

            double adjustedVelocity = physicsVelocity - robotSpeedTowardsGoal;

            setShooterState(adjustedVelocity, desiredAngleDeg);
        }

        private void setShooterState(double velocityMetersPerSec, double angleDeg) {
            currentTargetRPM = velocityMetersPerSec * RPM_PER_METER_PER_SECOND;

            double servoRange = SERVO_MAX_POS - SERVO_MIN_POS;
            double angleRange = ANGLE_AT_MAX - ANGLE_AT_MIN;
            double servoPos = ((angleDeg - ANGLE_AT_MIN) / angleRange) * servoRange + SERVO_MIN_POS;

            currentTargetRPM = Range.clip(currentTargetRPM, 0, 5000);
            servoPos = Range.clip(servoPos, SERVO_MIN_POS, SERVO_MAX_POS);

            shooterMotor.setVelocity(currentTargetRPM);
            angleServo.setPosition(servoPos);
            currentTargetAngle = angleDeg;
        }

        public void startShooter() { shooterActive = true; }

        public void stopShooter() {
            shooterActive = false;
            shooterMotor.setVelocity(0);
            feederServo.setPosition(FEEDER_IDLE);
        }

        public void feed() { feederServo.setPosition(FEEDER_FEEDING); }

        public void retractFeeder() { feederServo.setPosition(FEEDER_IDLE); }

        public boolean isShooterReady() {
            return Math.abs(shooterMotor.getVelocity() - currentTargetRPM) < 200;
        }

        public void addShooterTelemetry(Telemetry telemetry) {
            telemetry.addLine("=== Shooter ===");
            telemetry.addData("Current RPM", String.format("%.0f", shooterMotor.getVelocity()));
            telemetry.addData("Target RPM", String.format("%.0f", currentTargetRPM));
            telemetry.addData("Shooter Ready", isShooterReady() ? "YES" : "NO");
            telemetry.addData("Current Angle", String.format("%.1f", currentTargetAngle));
            telemetry.addData("Feeder Pos", String.format("%.2f", feederServo.getPosition()));
            telemetry.addData("Angle Servo Pos", String.format("%.2f", angleServo.getPosition()));
        }
    }
}

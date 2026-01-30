package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.limelight.LimelightControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.spindexer.Shooter;

@Autonomous(name = "Blue", group = "Blue", preselectTeleOp = "Drive")
public class Blue extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Subsystems
    private Shooter shooter;
    private SpindexerAuto spindexerauto;
    private LimelightControl limelight;
    private DcMotor intake;
    private boolean shootingRequested = false;


    // Hardware Names
    private static final String SPINDEXER_MOTOR = "spindexer";
    private static final String COLOR_SENSOR = "intakeColor";
    private static final String INTAKE_MOTOR = "intake";
    private static final String FEEDER_SERVO = "feeder";
    private static final String SHOOTER_MOTOR = "shooter";

    // Poses
    private final Pose startPose = new Pose(26, 129, Math.toRadians(144));
    private final Pose shootPose = new Pose(44, 104, Math.toRadians(144));
    private final Pose intake1Pose = new Pose(39, 94, Math.toRadians(180));
    private final Pose feed1Pose = new Pose(25, 94, Math.toRadians(180));
    private final Pose intake2Pose = new Pose(39, 48, Math.toRadians(180));
    private final Pose feed2Pose = new Pose(25, 48, Math.toRadians(180));

    // Paths
    private PathChain toShoot1, intake1, feed1, toShoot2, intake2, feed2, toShoot3;

    // Game Pattern
    private SpindexerAuto.GamePattern detectedPattern = SpindexerAuto.GamePattern.GREEN_FIRST;

    // Timeout for stuck paths (ms)
    private static final long PATH_TIMEOUT = 4000;

    public void buildPaths() {
        toShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144))
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        feed1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, feed1Pose))
                .setTangentHeadingInterpolation()
                .build();

        toShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(feed1Pose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        feed2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, feed2Pose))
                .setTangentHeadingInterpolation()
                .build();

        toShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(feed2Pose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (pathTimer.getElapsedTimeSeconds() < 0.05) {
                    follower.followPath(toShoot1);
                }

                if (!follower.isBusy()) {
                    spindexerauto.setModeShooting();
                    setPathState(1);
                }
                break;

            case 1: // Shooting 1
                // Wait for SpindexerAuto to finish shooting
                if (spindexerauto.getMode() == SpindexerAuto.SpindexerMode.INTAKING) {
                    // Shooting finished, move to Intake 1
                    intake.setPower(1.0);
                    follower.followPath(intake1, true);
                    setPathState(2);
                }
                break;

            case 2: // Intake 1 -> Feed 1
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.3);
                    follower.followPath(feed1, true);
                    setPathState(3);
                }
                break;

            case 3: // Feed 1 -> Shoot 2
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(toShoot2, true);
                    setPathState(4);
                }
                break;

            case 4: // Shooting 2
                if (!follower.isBusy() && !shootingRequested) {
                        spindexerauto.setModeShooting();
                    intake.setPower(0);
                    shootingRequested = true;
                }

                // ONLY leave after shooting truly finished
                if (shootingRequested &&
                        spindexerauto.getMode() == SpindexerAuto.SpindexerMode.INTAKING) {

                    shootingRequested = false;
                    intake.setPower(1.0);
                    follower.followPath(intake2, true);
                    setPathState(5);
                }
                break;

            case 5: // Intake 2 -> Feed 2
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.3);
                    follower.followPath(feed2, true);
                    setPathState(6);
                }
                break;

            case 6: // Feed 2 -> Shoot 3
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(toShoot3, true);
                    setPathState(7);
                }
                break;

            case 7: // Shooting 3
                if (!follower.isBusy() && !shootingRequested) {
                        spindexerauto.setModeShooting();
                    intake.setPower(0);
                    shootingRequested = true;
                }

                if (shootingRequested &&
                        spindexerauto.getMode() == SpindexerAuto.SpindexerMode.INTAKING) {

                    shootingRequested = false;
                    setPathState(8);
                }
                break;


            case 8: // End / Park
                if (!follower.isBusy()) {
                    setPathState(-1); // finished autonomous
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        spindexerauto.updateIntake();
        spindexerauto.updateAutoShoot();
        shooter.updateShooter();

        autonomousPathUpdate();

        telemetry.addLine("----- SPINDEXER AUTO -----");
        telemetry.addData("Mode", spindexerauto.getMode());
        telemetry.addData("Balls Shot", spindexerauto.getBallsShotCount());
        telemetry.addLine("----- SLOTS -----");
        telemetry.addData("Slot 0", spindexerauto.getSlotColor(0));
        telemetry.addData("Slot 1", spindexerauto.getSlotColor(1));
        telemetry.addData("Slot 2", spindexerauto.getSlotColor(2));
        telemetry.addData("pathState",pathState);
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        DcMotorEx spinMotor = hardwareMap.get(DcMotorEx.class, SPINDEXER_MOTOR);
        NormalizedColorSensor intakeSensor = hardwareMap.get(NormalizedColorSensor.class, COLOR_SENSOR);
        Servo feederServo = hardwareMap.get(Servo.class, FEEDER_SERVO);
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter = new Shooter(shooterMotor, feederServo);
        spindexerauto = new SpindexerAuto(this, spinMotor, intakeSensor, shooter);
        intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);

        limelight = new LimelightControl(hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {
        SpindexerAuto.GamePattern livePattern = limelight.getGamePatternFromTagsAuto();
        if (livePattern != null) {
            detectedPattern = livePattern;
        }
        telemetry.addData("Detected Pattern", livePattern != null ? livePattern : "Scanning...");
        telemetry.addData("Default Pattern", detectedPattern);
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        spindexerauto.setGamePattern(detectedPattern);
        limelight.stop();
        intake.setPower(1.0);
        spindexerauto.scanSlots();
        setPathState(0);
    }

    @Override
    public void stop() {
        shooter.stopShooter();
    }
}

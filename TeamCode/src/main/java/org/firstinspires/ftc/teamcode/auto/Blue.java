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
import org.firstinspires.ftc.teamcode.spindexer.Spindexer;

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

    // Hardware Names
    private static final String spindexerauto_MOTOR = "spindexer";
    private static final String COLOR_SENSOR = "intakeColor";
    private static final String INTAKE_MOTOR = "intake";
    private static final String FEEDER_SERVO = "feeder";
    private static final String SHOOTER_MOTOR = "shooter";

    // Poses - Extracted from User's Path Data
    private final Pose startPose = new Pose(19, 119, Math.toRadians(144)); // Start from Path1
    private final Pose shootPose = new Pose(44, 104.000, Math.toRadians(144));
    private final Pose intake1Pose = new Pose(44, 84, Math.toRadians(180));
    private final Pose feed1Pose = new Pose(22, 84, Math.toRadians(180));
    private final Pose intake2Pose = new Pose(44, 60, Math.toRadians(180));
    private final Pose feed2Pose = new Pose(22, 60.000, Math.toRadians(180));

    // Paths
    private PathChain toShoot1, intake1, feed1, toShoot2, intake2, feed2, toShoot3;

    // Pattern
    private SpindexerAuto.GamePattern detectedPattern = SpindexerAuto.GamePattern.GREEN_FIRST; // Default

    public void buildPaths() {
        // Path1: Start -> Shoot 1
        toShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144))
                .build();

        // Path2: Shoot 1 -> Intake 1
        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        // Path3: Intake 1 -> Feed 1
        feed1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, feed1Pose))
                .setTangentHeadingInterpolation()
                .build();

        // Path4: Feed 1 -> Shoot 2
        toShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(feed1Pose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                .build();

        // Path5: Shoot 2 -> Intake 2
        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        // Path6: Intake 2 -> Feed 2
        feed2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, feed2Pose))
                .setTangentHeadingInterpolation()
                .build();

        // Path7: Feed 2 -> Shoot 3
        toShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(feed2Pose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start -> Shoot 1
                follower.followPath(toShoot1);
                setPathState(1);
                break;

            case 1: // Moving to Shoot 1
                if (!follower.isBusy()) {
                    /* Start Shooting Sequence */
                    spindexerauto.setModeShooting();
                    spindexerauto.spinUpShooter(); // Ensure shooter is spinning
                    setPathState(2);
                }
                break;

            case 2: // Shooting 1 (3 balls)
                // Spindexer handles logic. Wait until it switches back to INTAKING
                // Keep spinning up shooter
                spindexerauto.spinUpShooter();

                if (spindexerauto.getMode() == SpindexerAuto.SpindexerMode.INTAKING) {
                    // Done shooting
                    spindexerauto.stopShooter();

                    // Start next path: Intake 1
                    follower.followPath(intake1, true); // Hold end?
                    intake.setPower(1.0); // Start intake
                    setPathState(3);
                }
                break;

            case 3: // Moving to Intake 1
                if (!follower.isBusy()) {
                    /* At Intake 1 */
                    // Move to Feed 1
                    follower.setMaxPower(0.5);
                    follower.followPath(feed1, true);
                    setPathState(4);
                }
                break;

            case 4: // Moving to Feed 1
                // We keep intake running
                if (!follower.isBusy()) {
                    /* At Feed 1 */
                    // Prepare to go back to shoot
                    // Move to Shoot 2
                    follower.setMaxPower(1);
                    follower.followPath(toShoot2, true);
                    setPathState(5);
                }
                break;

            case 5: // Moving to Shoot 2
                if (!follower.isBusy()) {
                    /* Start Shooting Sequence 2 */
                    spindexerauto.setModeShooting();
                    spindexerauto.spinUpShooter();
                    setPathState(6);
                }
                break;

            case 6: // Shooting 2
                spindexerauto.spinUpShooter();
                if (spindexerauto.getMode() == SpindexerAuto.SpindexerMode.INTAKING) {
                    spindexerauto.stopShooter();

                    // Start next cycle: Intake 2
                    follower.followPath(intake2, true);
                    intake.setPower(1.0);
                    setPathState(7);
                }
                break;

            case 7: // Moving to Intake 2
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.5);
                    follower.followPath(feed2, true);
                    setPathState(8);
                }
                break;

            case 8: // Moving to Feed 2
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(toShoot3, true);
                    setPathState(9);
                }
                break;

            case 9: // Moving to Shoot 3
                if (!follower.isBusy()) {
                    /* Start Shooting Sequence 3 */
                    spindexerauto.setModeShooting();
                    spindexerauto.spinUpShooter();
                    setPathState(10);
                }
                break;

            case 10: // Shooting 3
                spindexerauto.spinUpShooter();
                if (spindexerauto.getMode() == SpindexerAuto.SpindexerMode.INTAKING) {
                    spindexerauto.stopShooter();
                    setPathState(11);
                }
                break;

            case 11: // Moving to Park
                if (!follower.isBusy()) {
                    setPathState(-1); // End
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

        telemetry.addLine("----- spindexerauto -----");
        telemetry.addData("Mode", spindexerauto.getMode());
        telemetry.addData("Balls Shot", spindexerauto.getBallsShotCount());
        telemetry.addLine("----- SLOTS -----");
        telemetry.addData("Slot 0", spindexerauto.getSlotColor(0));
        telemetry.addData("Slot 1", spindexerauto.getSlotColor(1));
        telemetry.addData("Slot 2", spindexerauto.getSlotColor(2));

    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        // Initialize Subsystems
        DcMotorEx spinMotor = hardwareMap.get(DcMotorEx.class, spindexerauto_MOTOR);
        NormalizedColorSensor intakeSensor = hardwareMap.get(NormalizedColorSensor.class, COLOR_SENSOR);
        Servo feederServo = hardwareMap.get(Servo.class, FEEDER_SERVO);
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter = new Shooter(shooterMotor, feederServo);
        spindexerauto = new SpindexerAuto(this, spinMotor, intakeSensor, shooter);
        intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);

        // Limelight
        limelight = new LimelightControl(hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {
        // Continuously check for pattern
        SpindexerAuto.GamePattern livePattern = limelight.getGamePatternFromTagsAuto();
        if (livePattern != null) {
            detectedPattern = livePattern;
            telemetry.addData("DETECTED PATTERN", detectedPattern);
        } else {
            telemetry.addData("DETECTED PATTERN", "Scanning...");
            telemetry.addData("Defaulting To", detectedPattern);
        }
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        // Lock in pattern
        spindexerauto.setGamePattern(detectedPattern);
        limelight.stop();
        intake.setPower(1.0);
        spindexerauto.scanSlots(); // Scan for pre-loaded balls
        setPathState(0);
    }

    @Override
    public void stop() {
        shooter.stopShooter();
    }
}
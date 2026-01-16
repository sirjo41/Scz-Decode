package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.limelight.LimelightControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.spindexer.Spindexer;

/**
 * Advanced State-Machine Based Autonomous
 * Features: Concurrent operations, timeout protection, dynamic time management
 */
// @Autonomous(name = "Red_1_Advanced", group = "Autonomous") // DISABLED
public class Red_1 extends LinearOpMode {

    // ======================== STATE MACHINE ========================

    private enum AutoState {
        INIT,
        MOVE_TO_SCORE_PRELOAD,
        SHOOT_PRELOAD,
        MOVE_TO_PICKUP_1,
        INTAKE_1,
        MOVE_TO_SCORE_1,
        SHOOT_1,
        MOVE_TO_PICKUP_2,
        INTAKE_2,
        MOVE_TO_SCORE_2,
        SHOOT_2,
        MOVE_TO_PICKUP_3,
        INTAKE_3,
        MOVE_TO_SCORE_3,
        SHOOT_3,
        PARK,
        COMPLETE
    }

    // ======================== CONFIGURATION ========================

    // Autonomous time limits
    private static final double AUTO_DURATION_SECONDS = 30.0;
    private static final double CRITICAL_TIME_REMAINING = 10.0; // Skip low-priority actions
    private static final double EMERGENCY_TIME_REMAINING = 5.0; // Only essential actions

    // Timeout configurations (seconds)
    private static final double PATH_TIMEOUT = 5.0;
    private static final double INTAKE_TIMEOUT = 3.0;
    private static final double SHOOT_TIMEOUT = 5.0;
    private static final double SHOOTER_SPINUP_TIME = 0.5;

    // ======================== HARDWARE ========================

    private Follower follower;
    private Spindexer spindexer;
    private LimelightControl limelight;

    // ======================== STATE VARIABLES ========================

    private AutoState currentState = AutoState.INIT;
    private Timer stateTimer, autoTimer;
    private Spindexer.GamePattern detectedPattern = Spindexer.GamePattern.GREEN_FIRST;
    private int cyclesCompleted = 0;

    // Performance tracking
    private long loopCount = 0;
    private double avgLoopTime = 0;

    // ======================== PATHS ========================

    private final Pose startPose = new Pose(72, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(98.916, 101.607, Math.toRadians(42));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    // ======================== MAIN ========================

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        buildPaths();
        initLoop(); // Pattern detection

        waitForStart();

        if (isStopRequested())
            return;

        // START AUTONOMOUS
        autoTimer.resetTimer();
        stateTimer.resetTimer();
        limelight.stop(); // Save resources
        spindexer.setGamePattern(detectedPattern);

        transitionTo(AutoState.MOVE_TO_SCORE_PRELOAD);

        // Main state machine loop
        while (opModeIsActive() && currentState != AutoState.COMPLETE) {
            long loopStart = System.nanoTime();

            updateStateMachine();
            updateTelemetry();

            // Calculate loop performance
            long loopEnd = System.nanoTime();
            double loopTime = (loopEnd - loopStart) / 1_000_000.0; // Convert to ms
            avgLoopTime = (avgLoopTime * loopCount + loopTime) / (loopCount + 1);
            loopCount++;
        }

        telemetry.addData("Status", "Autonomous Complete!");
        telemetry.addData("Cycles", cyclesCompleted);
        telemetry.update();
    }

    // ======================== INITIALIZATION ========================

    private void initializeHardware() {
        stateTimer = new Timer();
        autoTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        try {
            spindexer = new Spindexer(
                    this,
                    hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "spindexer"),
                    hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "intakeColor"),
                    hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "shooterMotor"),
                    hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "feederServo"),
                    hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "shooterColor"));

            limelight = new LimelightControl(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Init Error", e.getMessage());
            telemetry.update();
        }
    }

    private void initLoop() {
        Pose limelightPose = null;

        while (!isStarted() && !isStopRequested()) {
            // Pattern detection
            Spindexer.GamePattern tagPattern = limelight.getGamePatternFromTags();
            if (tagPattern != null) {
                detectedPattern = tagPattern;
                telemetry.addData("✓ Pattern", detectedPattern);
            } else {
                telemetry.addData("⏳ Pattern", "Searching... (Default: " + detectedPattern + ")");
            }

            // Localization
            Pose pose = limelight.getRobotPose();
            if (pose != null) {
                limelightPose = pose;
                telemetry.addData("✓ Limelight", "X:%.1f Y:%.1f H:%.0f°",
                        pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
            } else {
                telemetry.addData("⏳ Limelight", "Searching...");
            }

            telemetry.addLine("\n--- Camera Info ---");
            limelight.telemetry();
            telemetry.update();
        }

        // Apply localization if available
        if (limelightPose != null) {
            follower.setStartingPose(limelightPose);
            telemetry.log().add("✓ Relocalized to Limelight Pose");
        } else {
            telemetry.log().add("⚠ Using Default Start Pose");
        }
    }

    // ======================== STATE MACHINE ========================

    private void updateStateMachine() {
        double timeRemaining = AUTO_DURATION_SECONDS - autoTimer.getElapsedTimeSeconds();
        double stateElapsed = stateTimer.getElapsedTimeSeconds();

        switch (currentState) {
            case MOVE_TO_SCORE_PRELOAD:
                if (stateElapsed < 0.1) {
                    follower.followPath(scorePreload);
                }
                follower.update();

                if (!follower.isBusy() || stateElapsed > PATH_TIMEOUT) {
                    transitionTo(AutoState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                if (stateElapsed < 0.1) {
                    follower.holdPoint(scorePose);
                    spindexer.startShooter();
                }
                follower.update();

                // Wait for shooter to spin up, then shoot
                if (stateElapsed > SHOOTER_SPINUP_TIME && spindexer.isShooterReady()) {
                    spindexer.ejectAllByPattern(telemetry);
                    cyclesCompleted++;

                    // Decide next action based on time
                    if (timeRemaining > CRITICAL_TIME_REMAINING) {
                        transitionTo(AutoState.MOVE_TO_PICKUP_1);
                    } else {
                        transitionTo(AutoState.PARK);
                    }
                } else if (stateElapsed > SHOOT_TIMEOUT) {
                    // Timeout - skip and move on
                    transitionTo(AutoState.MOVE_TO_PICKUP_1);
                }
                break;

            case MOVE_TO_PICKUP_1:
                if (stateElapsed < 0.1) {
                    follower.followPath(grabPickup1, true);
                    spindexer.startIntake(); // Start intake while moving!
                }
                follower.update();

                // Check intake progress
                if (spindexer.isIntakeComplete()) {
                    spindexer.completeIntake();
                }

                if (!follower.isBusy() || stateElapsed > PATH_TIMEOUT) {
                    transitionTo(AutoState.INTAKE_1);
                }
                break;

            case INTAKE_1:
                // Intake might already be done from concurrent operation
                if (spindexer.isIntakeComplete()) {
                    spindexer.completeIntake();
                    transitionTo(AutoState.MOVE_TO_SCORE_1);
                } else if (stateElapsed > INTAKE_TIMEOUT) {
                    // Timeout - cancel and move on
                    spindexer.cancelIntake();
                    transitionTo(AutoState.MOVE_TO_SCORE_1);
                }
                break;

            case MOVE_TO_SCORE_1:
                if (stateElapsed < 0.1) {
                    follower.followPath(scorePickup1, true);
                    spindexer.startShooter(); // Pre-spin shooter while moving!
                }
                follower.update();

                if (!follower.isBusy() || stateElapsed > PATH_TIMEOUT) {
                    transitionTo(AutoState.SHOOT_1);
                }
                break;

            case SHOOT_1:
                if (stateElapsed < 0.1) {
                    follower.holdPoint(scorePose);
                }
                follower.update();

                if (spindexer.isShooterReady()) {
                    spindexer.ejectAllByPattern(telemetry);
                    cyclesCompleted++;

                    // Decide if we have time for cycle 2
                    if (timeRemaining > CRITICAL_TIME_REMAINING) {
                        transitionTo(AutoState.MOVE_TO_PICKUP_2);
                    } else {
                        transitionTo(AutoState.PARK);
                    }
                } else if (stateElapsed > SHOOT_TIMEOUT) {
                    transitionTo(AutoState.MOVE_TO_PICKUP_2);
                }
                break;

            case MOVE_TO_PICKUP_2:
                if (stateElapsed < 0.1) {
                    follower.followPath(grabPickup2, true);
                    spindexer.startIntake();
                }
                follower.update();

                if (spindexer.isIntakeComplete()) {
                    spindexer.completeIntake();
                }

                if (!follower.isBusy() || stateElapsed > PATH_TIMEOUT) {
                    transitionTo(AutoState.INTAKE_2);
                }
                break;

            case INTAKE_2:
                if (spindexer.isIntakeComplete()) {
                    spindexer.completeIntake();
                    transitionTo(AutoState.MOVE_TO_SCORE_2);
                } else if (stateElapsed > INTAKE_TIMEOUT) {
                    spindexer.cancelIntake();
                    transitionTo(AutoState.MOVE_TO_SCORE_2);
                }
                break;

            case MOVE_TO_SCORE_2:
                if (stateElapsed < 0.1) {
                    follower.followPath(scorePickup2, true);
                    spindexer.startShooter();
                }
                follower.update();

                if (!follower.isBusy() || stateElapsed > PATH_TIMEOUT) {
                    transitionTo(AutoState.SHOOT_2);
                }
                break;

            case SHOOT_2:
                if (stateElapsed < 0.1) {
                    follower.holdPoint(scorePose);
                }
                follower.update();

                if (spindexer.isShooterReady()) {
                    spindexer.ejectAllByPattern(telemetry);
                    cyclesCompleted++;

                    // Decide if we have time for cycle 3
                    if (timeRemaining > EMERGENCY_TIME_REMAINING) {
                        transitionTo(AutoState.MOVE_TO_PICKUP_3);
                    } else {
                        transitionTo(AutoState.PARK);
                    }
                } else if (stateElapsed > SHOOT_TIMEOUT) {
                    transitionTo(AutoState.PARK);
                }
                break;

            case MOVE_TO_PICKUP_3:
                if (stateElapsed < 0.1) {
                    follower.followPath(grabPickup3, true);
                    spindexer.startIntake();
                }
                follower.update();

                if (spindexer.isIntakeComplete()) {
                    spindexer.completeIntake();
                }

                if (!follower.isBusy() || stateElapsed > PATH_TIMEOUT) {
                    transitionTo(AutoState.INTAKE_3);
                }
                break;

            case INTAKE_3:
                if (spindexer.isIntakeComplete()) {
                    spindexer.completeIntake();
                    transitionTo(AutoState.MOVE_TO_SCORE_3);
                } else if (stateElapsed > INTAKE_TIMEOUT) {
                    spindexer.cancelIntake();
                    transitionTo(AutoState.MOVE_TO_SCORE_3);
                }
                break;

            case MOVE_TO_SCORE_3:
                if (stateElapsed < 0.1) {
                    follower.followPath(scorePickup3, true);
                    spindexer.startShooter();
                }
                follower.update();

                if (!follower.isBusy() || stateElapsed > PATH_TIMEOUT) {
                    transitionTo(AutoState.SHOOT_3);
                }
                break;

            case SHOOT_3:
                if (stateElapsed < 0.1) {
                    follower.holdPoint(scorePose);
                }
                follower.update();

                if (spindexer.isShooterReady()) {
                    spindexer.ejectAllByPattern(telemetry);
                    cyclesCompleted++;
                    transitionTo(AutoState.PARK);
                } else if (stateElapsed > SHOOT_TIMEOUT) {
                    transitionTo(AutoState.PARK);
                }
                break;

            case PARK:
                // Could add parking logic here if needed
                transitionTo(AutoState.COMPLETE);
                break;

            case COMPLETE:
                spindexer.stopShooter();
                spindexer.emergencyStop();
                break;
        }
    }

    private void transitionTo(AutoState newState) {
        currentState = newState;
        stateTimer.resetTimer();
    }

    // ======================== TELEMETRY ========================

    private void updateTelemetry() {
        double timeRemaining = AUTO_DURATION_SECONDS - autoTimer.getElapsedTimeSeconds();

        // Header
        telemetry.addLine("=== ADVANCED AUTONOMOUS ===");
        telemetry.addData("State", currentState);
        telemetry.addData("Time", "%.1fs / %.1fs", autoTimer.getElapsedTimeSeconds(), AUTO_DURATION_SECONDS);
        telemetry.addData("Cycles", cyclesCompleted);

        // Progress bar
        int barLength = 20;
        int filled = (int) ((autoTimer.getElapsedTimeSeconds() / AUTO_DURATION_SECONDS) * barLength);
        StringBuilder bar = new StringBuilder("[");
        for (int i = 0; i < barLength; i++) {
            bar.append(i < filled ? "█" : "░");
        }
        bar.append("]");
        telemetry.addData("Progress", bar.toString());

        // Subsystem status
        telemetry.addLine("\n--- Subsystems ---");
        telemetry.addData("Shooter", "%.0f%% (%.0f tps)",
                spindexer.getShooterVelocityPercent(),
                spindexer.getShooterVelocity());
        telemetry.addData("Indexer", spindexer.isIndexerBusy() ? "Moving" : "Stopped");

        Spindexer.Ball[] slots = spindexer.getSlots();
        telemetry.addData("Balls", "%s | %s | %s", slots[0], slots[1], slots[2]);

        // Performance
        telemetry.addLine("\n--- Performance ---");
        telemetry.addData("Loop Time", "%.1fms (avg)", avgLoopTime);
        telemetry.addData("FPS", "%.0f", 1000.0 / Math.max(avgLoopTime, 1.0));

        // Warnings
        if (timeRemaining < EMERGENCY_TIME_REMAINING) {
            telemetry.addLine("\n⚠ EMERGENCY TIME - ESSENTIAL ONLY");
        } else if (timeRemaining < CRITICAL_TIME_REMAINING) {
            telemetry.addLine("\n⚠ LOW TIME - PRIORITIZING");
        }

        telemetry.update();
    }

    // ======================== PATH BUILDING ========================

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, new Pose(102.056, 83.215, Math.toRadians(0))))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(0))
                .addPath(new BezierLine(new Pose(102.056, 83.215), new Pose(127.626, 83.215)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(127.626, 83.215), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, new Pose(102.056, 60.336, Math.toRadians(0))))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(0))
                .addPath(new BezierLine(new Pose(102.056, 60.336), new Pose(124.710, 60.336)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(124.710, 60.336), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, new Pose(102.056, 37.5, Math.toRadians(0))))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(0))
                .addPath(new BezierLine(new Pose(102.056, 37.5), new Pose(124.710, 37.5)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(124.710, 37.5), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), scorePose.getHeading())
                .build();
    }
}

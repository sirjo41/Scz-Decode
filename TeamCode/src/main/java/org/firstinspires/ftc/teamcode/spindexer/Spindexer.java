package org.firstinspires.ftc.teamcode.spindexer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Spindexer - Simplified rotary indexer for left/right movement.
 */
public class Spindexer {

    // Motor and encoder constants
    private static final double CPR_MOTOR = 751.8;
    private static final double GEAR_RATIO = 1.0;
    private static final double CPR_PLATE = CPR_MOTOR / GEAR_RATIO;

    // Rotation constants
    private static final double TICKS_PER_SLOT = CPR_PLATE / 3.0;

    // PID coefficients for position control
    private static final PIDFCoefficients POS_PIDF = new PIDFCoefficients(
            8.0,
            0.0,
            0.6,
            0.0);

    // Motor control parameters
    private static final int TARGET_TOL = 2;
    private static final double MAX_POWER = 0.6;

    /**
     * Game pattern enumeration.
     */
    public enum GamePattern {
        GREEN_FIRST,
        GREEN_SECOND,
        GREEN_THIRD
    }

    // Hardware components
    private final DcMotorEx motor;
    private final LinearOpMode opMode;

    // State tracking
    private int zeroCount = 0;
    private double accum = 0.0;
    private int targetCounts = 0;

    // Game pattern
    private GamePattern pattern = GamePattern.GREEN_FIRST;

    /**
     * Constructor - Initializes the spindexer with motor.
     */
    public Spindexer(LinearOpMode opMode, DcMotorEx motor) {
        this.opMode = opMode;
        this.motor = motor;

        // Configure motor for position control
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Apply PID coefficients
        try {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, POS_PIDF);
        } catch (Exception ignored) {
        }
        motor.setTargetPositionTolerance(TARGET_TOL);

        // Initialize zero position
        zeroCount = motor.getCurrentPosition();
        accum = 0.0;
        targetCounts = motor.getCurrentPosition();
        motor.setTargetPosition(targetCounts);
        motor.setPower(MAX_POWER);
    }

    /**
     * Move left (counter-clockwise) by one slot.
     */
    public void moveLeft(Telemetry telemetry) {
        accum -= TICKS_PER_SLOT;
        int target = (int) Math.rint(zeroCount + accum);
        goTo(target, telemetry);
    }

    /**
     * Move right (clockwise) by one slot.
     */
    public void moveRight(Telemetry telemetry) {
        accum += TICKS_PER_SLOT;
        int target = (int) Math.rint(zeroCount + accum);
        goTo(target, telemetry);
    }

    /**
     * Gets the current game pattern.
     */
    public GamePattern getGamePattern() {
        return pattern;
    }

    /**
     * Sets the game pattern.
     */
    public void setGamePattern(GamePattern pattern) {
        this.pattern = pattern;
    }

    /**
     * Returns the current encoder position.
     */
    public int getEncoder() {
        return motor.getCurrentPosition();
    }

    /**
     * Returns the current target encoder position.
     */
    public int getTarget() {
        return targetCounts;
    }

    /**
     * Resets the zero reference to the current position.
     */
    public void rezeroHere() {
        zeroCount = motor.getCurrentPosition();
        accum = 0.0;
        setTarget(motor.getCurrentPosition());
    }

    /**
     * Sets the target position and configures motor for movement.
     */
    private void setTarget(int t) {
        targetCounts = t;
        motor.setTargetPosition(targetCounts);
        if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        motor.setPower(MAX_POWER);
    }

    /**
     * Moves to target position and blocks until complete.
     */
    private void goTo(int t, Telemetry telemetry) {
        setTarget(t);
        while (opModeIsActive() && motor.isBusy()) {
            motor.setPower(MAX_POWER);

            if (telemetry != null) {
                telemetry.addData("enc", motor.getCurrentPosition());
                telemetry.addData("target", targetCounts);
                telemetry.addData("pattern", pattern);
                telemetry.update();
            }
        }
    }

    /**
     * Checks if the OpMode is still active.
     */
    private boolean opModeIsActive() {
        return opMode.opModeIsActive();
    }

    /**
     * Emergency stop - stops motor immediately.
     */
    public void emergencyStop() {
        motor.setPower(0);
    }
}

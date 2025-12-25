package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Spindexer - A 3-slot rotary indexer for ball storage and sorting.
 * Manages intake, storage, and selective ejection of colored balls.
 */
public class Spindexer {

    // Motor and encoder constants
    private static final double CPR_MOTOR = 751.8;
    private static final double GEAR_RATIO = 1.0;
    private static final double CPR_PLATE = CPR_MOTOR / GEAR_RATIO;

    // Rotation constants (plate has 3 slots: 120° each, 60° half-steps)
    private static final double TICKS_PER_SLOT = CPR_PLATE / 3.0;
    private static final double TICKS_PER_60 = CPR_PLATE / 6.0;

    // PID coefficients for position control
    private static final PIDFCoefficients POS_PIDF = new PIDFCoefficients(
            8.0,   // kP - Proportional gain
            0.0,   // kI - Integral gain
            0.6,   // kD - Derivative gain
            0.0    // kF - Feedforward gain
    );

    // Motor control parameters
    private static final int TARGET_TOL = 2;
    private static final double MAX_POWER = 0.6;

    // Color sensor threshold (tune based on lighting conditions)
    private static final int PRESENCE_ALPHA_THRESHOLD = 5;

    /**
     * Ball color enumeration
     */
    public enum Ball {
        EMPTY,
        PURPLE,
        GREEN
    }

    // Hardware components
    private final DcMotorEx motor;
    private final ColorSensor intakeColor;

    // State tracking variables
    private int zeroCount = 0;
    private double accum = 0.0;
    private int targetCounts = 0;
    private int intakeIndex = 0;

    // Ball storage (3 slots)
    private final Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};

    /**
     * Constructor - Initializes the spindexer with motor and color sensor.
     *
     * @param motor       The motor controlling the indexer rotation
     * @param intakeColor Color sensor at the intake position
     */
    public Spindexer(DcMotorEx motor, ColorSensor intakeColor) {
        this.motor = motor;
        this.intakeColor = intakeColor;

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
     * Intakes one ball: reads color, stores it, and rotates to next slot.
     *
     * @param telemetry Telemetry object for debug output
     */
    public void intakeOne(Telemetry telemetry) {
        Ball color = readColorAtIntake();
        slots[intakeIndex] = color;

        // Rotate 120° to next intake position
        move120NoEject(telemetry);

        intakeIndex = mod3(intakeIndex + 1);
    }

    /**
     * Ejects all balls with priority: GREEN first, then all PURPLE balls.
     *
     * @param telemetry Telemetry object for debug output
     */
    public void ejectAllGreenThenPurple(Telemetry telemetry) {
        // Eject green ball if present
        Integer greenIdx = findFirst(Ball.GREEN);
        if (greenIdx != null) {
            ejectSlot(greenIdx, telemetry);
        }

        // Eject all purple balls
        for (int i = 0; i < 3; i++) {
            if (slots[i] == Ball.PURPLE) {
                ejectSlot(i, telemetry);
            }
        }
    }

    /**
     * Resets the zero reference to the current position.
     * Clears angular reference but not ball inventory.
     */
    public void rezeroHere() {
        zeroCount = motor.getCurrentPosition();
        accum = 0.0;
        setTarget(motor.getCurrentPosition());
        intakeIndex = 0;
    }

    /**
     * Returns a copy of the current slot states.
     */
    public Ball[] getSlots() {
        return slots.clone();
    }

    /**
     * Returns the current intake slot index.
     */
    public int getIntakeIndex() {
        return intakeIndex;
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
     * Ejects a specific slot by aligning it to the eject position.
     *
     * @param slotIndex Absolute slot index (0-2) to eject
     * @param telemetry Telemetry object for debug output
     */
    private void ejectSlot(int slotIndex, Telemetry telemetry) {
        // Align so that slot will be at eject position after +60° half-step
        int deltaSlots = mod3(slotIndex - 1 - intakeIndex);
        for (int k = 0; k < deltaSlots; k++) {
            move120NoEject(telemetry);
            intakeIndex = mod3(intakeIndex + 1);
        }

        // Move +60° to eject position
        int mid = stepForward60();
        goTo(mid, telemetry);

        // TODO: Add ejector mechanism actuation here (servo/motor)
        // Example: ejectServo.setPosition(EJECT_POSITION);

        // Clear the slot
        slots[slotIndex] = Ball.EMPTY;

        // Complete rotation to next intake stop (+60°)
        int nextStop = stepForward60();
        goTo(nextStop, telemetry);

        intakeIndex = mod3(intakeIndex + 1);
    }

    /**
     * Finds the first occurrence of a specific ball color.
     *
     * @param color The ball color to search for
     * @return Index of the first matching slot, or null if not found
     */
    private Integer findFirst(Ball color) {
        for (int i = 0; i < 3; i++) {
            if (slots[i] == color) {
                return i;
            }
        }
        return null;
    }

    /**
     * Modulo 3 operation that handles negative numbers correctly.
     */
    private int mod3(int x) {
        return ((x % 3) + 3) % 3;
    }

    /**
     * Calculates target position for a 60° forward step.
     *
     * @return Target encoder count
     */
    private int stepForward60() {
        accum += TICKS_PER_60;
        return (int) Math.rint(zeroCount + accum);
    }

    /**
     * Rotates 120° forward in two 60° half-steps without ejecting.
     *
     * @param telemetry Telemetry object for debug output
     */
    private void move120NoEject(Telemetry telemetry) {
        int mid = stepForward60();
        goTo(mid, telemetry);
        int stop = stepForward60();
        goTo(stop, telemetry);
    }

    /**
     * Sets the target position and configures motor for movement.
     *
     * @param t Target encoder position
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
     *
     * @param t         Target encoder position
     * @param telemetry Telemetry object for debug output (can be null)
     */
    private void goTo(int t, Telemetry telemetry) {
        setTarget(t);
        while (opModeIsActive() && motor.isBusy()) {
            motor.setPower(MAX_POWER);

            // Display movement progress
            if (telemetry != null) {
                telemetry.addData("enc", motor.getCurrentPosition());
                telemetry.addData("target", targetCounts);
                telemetry.addData("intakeIndex", intakeIndex);
                telemetry.addData("slots", "%s | %s | %s", slots[0], slots[1], slots[2]);
                telemetry.update();
            }
        }
    }

    /**
     * Checks if a ball is present at the intake position.
     *
     * @return True if ball detected, false otherwise
     */
    private boolean ballPresentAtIntake() {
        int alpha = intakeColor.alpha();
        return alpha > PRESENCE_ALPHA_THRESHOLD;
    }

    /**
     * Reads and identifies the ball color at the intake position.
     *
     * @return Ball color (EMPTY, PURPLE, or GREEN)
     */
    private Ball readColorAtIntake() {
        // Return EMPTY if no ball detected
        if (!ballPresentAtIntake()) return Ball.EMPTY;

        int r = intakeColor.red();
        int g = intakeColor.green();
        int b = intakeColor.blue();

        // Color classification logic
        boolean isPurple = (b > g + 5) && (r > g + 5);
        boolean isGreen = (g > r + 5) && (g > b + 5);

        if (isGreen && !isPurple) return Ball.GREEN;
        if (isPurple && !isGreen) return Ball.PURPLE;

        // Fallback: choose dominant color channel
        if (g >= r && g >= b) return Ball.GREEN;
        return Ball.PURPLE;
    }

    /**
     * Checks if the OpMode is still active.
     * WARNING: This implementation always returns true.
     * In a real OpMode, inject a reference to LinearOpMode and check opModeIsActive().
     *
     * @return Always true (placeholder implementation)
     */
    private boolean opModeIsActive() {
        // TODO: Replace with actual OpMode.opModeIsActive() check
        // Consider passing LinearOpMode reference in constructor or movement methods
        return true;
    }
}
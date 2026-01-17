package org.firstinspires.ftc.teamcode.spindexer;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Spindexer - Simplified rotary indexer for left/right movement.
 */
@Configurable
public class Spindexer {

    // Motor and encoder constants
    private static final double CPR_MOTOR = 28; // TODO: Verify motor encoder CPR
    private static final double GEAR_RATIO = 1.0; // TODO: Tune based on actual gear ratio
    private static final double CPR_PLATE = CPR_MOTOR / GEAR_RATIO;

    // Rotation constants
    public static final double TICKS_PER_SLOT = 455;// TODO: Tune number of slots (currently 3)

    // PID coefficients for position control
    public static final PIDFCoefficients POS_PIDF = new PIDFCoefficients(
            16.0, // TODO: Tune P coefficient
            0.0, // TODO: Tune I coefficient
            3.0, // TODO: Tune D coefficient
            0.0); // TODO: Tune F coefficient

    // Motor control parameters
    private static final int TARGET_TOL = 2; // TODO: Tune position tolerance
    private static final double MAX_POWER = 1; // TODO: Tune max motor power

    /**
     * Game pattern enumeration.
     */
    public enum GamePattern {
        GREEN_FIRST,
        GREEN_SECOND,
        GREEN_THIRD
    }

    /**
     * Slot color enumeration for tracking game elements.
     */
    public enum SlotColor {
        EMPTY,
        PURPLE,
        GREEN,
        UNKNOWN
    }

    // Hardware components
    private final DcMotorEx motor;
    private final ColorSensor colorSensor;
    private final LinearOpMode opMode;

    // State tracking
    private int zeroCount = 0;
    private double accum = 0.0;
    private int targetCounts = 0;

    // Intake state tracking
    private final SlotColor[] slots = new SlotColor[3];
    private int currentSlotIndex = 0;
    private boolean lastColorDetected = false;

    // Game pattern
    private GamePattern pattern = GamePattern.GREEN_FIRST;

    /**
     * Constructor - Initializes the spindexer with motor and intake system.
     */
    public Spindexer(LinearOpMode opMode, DcMotorEx motor,
            ColorSensor colorSensor) {
        this.opMode = opMode;
        this.motor = motor;
        this.colorSensor = colorSensor;

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

        // Initialize all slots as empty
        for (int i = 0; i < slots.length; i++) {
            slots[i] = SlotColor.EMPTY;
        }
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
     * Detects color from sensor RGB values.
     */
    private SlotColor detectColor() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        // TODO: Tune these color thresholds based on your sensor and lighting
        if (red > green && red > blue && red > 100) {
            return SlotColor.PURPLE;
        } else if (green > red && green > blue && green > 100) {
            return SlotColor.GREEN;
        } else if (red < 50 && green < 50 && blue < 50) {
            return SlotColor.EMPTY;
        }
        return SlotColor.UNKNOWN;
    }

    /**
     * Updates intake system - checks color sensor and stores color if object
     * detected.
     * Call this periodically in your main loop.
     */
    public void updateIntake() {
        SlotColor detectedColor = detectColor();

        // Detect rising edge - color changed from EMPTY/UNKNOWN to a valid color
        boolean objectDetected = (detectedColor == SlotColor.PURPLE || detectedColor == SlotColor.GREEN);

        if (objectDetected && !lastColorDetected) {
            // Store color in current slot and advance to next slot
            if (currentSlotIndex < slots.length) {
                slots[currentSlotIndex] = detectedColor;
                currentSlotIndex = (currentSlotIndex + 1) % slots.length;
            }
        }

        lastColorDetected = objectDetected;
    }

    /**
     * Returns the color in a specific slot (0-2).
     */
    public SlotColor getSlotColor(int slotIndex) {
        if (slotIndex >= 0 && slotIndex < slots.length) {
            return slots[slotIndex];
        }
        return SlotColor.EMPTY;
    }

    /**
     * Clears all slots (marks them as empty).
     */
    public void clearSlots() {
        for (int i = 0; i < slots.length; i++) {
            slots[i] = SlotColor.EMPTY;
        }
        currentSlotIndex = 0;
    }

    /**
     * Adds telemetry data for slot contents.
     */
    public void addSlotTelemetry(Telemetry telemetry) {
        telemetry.addLine("=== Slot Contents ===");
        for (int i = 0; i < slots.length; i++) {
            telemetry.addData("Slot " + i, slots[i]);
        }
        telemetry.addData("Next Slot", currentSlotIndex);
        telemetry.addData("Current Color", detectColor());
    }

    /**
     * Emergency stop - stops all motors immediately.
     */
    public void emergencyStop() {
        motor.setPower(0);
    }
}

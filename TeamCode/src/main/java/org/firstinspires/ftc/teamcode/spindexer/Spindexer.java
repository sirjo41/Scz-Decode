package org.firstinspires.ftc.teamcode.spindexer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
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
            8.0, // kP - Proportional gain
            0.0, // kI - Integral gain
            0.6, // kD - Derivative gain
            0.0 // kF - Feedforward gain
    );

    // Motor control parameters
    private static final int TARGET_TOL = 2;
    private static final double MAX_POWER = 0.6;

    // Color sensor threshold (tune based on lighting conditions)
    private static final int PRESENCE_ALPHA_THRESHOLD = 5;

    // Shooter constants
    private static final double SHOOTER_VELOCITY = 2000.0; // Ticks per second
    private static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(
            10.0, 0.0, 0.0, 12.0 // Basic PIDF; custom tune required
    );

    // Feeder Servo constants
    private static final double FEEDER_IDLE = 0.0;
    private static final double FEEDER_EJECT = 0.5;
    private static final int FEEDER_SLEEP_MS = 250; // Delay for servo actuation

    /**
     * Ball color enumeration
     */
    public enum Ball {
        EMPTY,
        PURPLE,
        GREEN
    }

    /**
     * Game pattern enumeration for ejection strategy
     * Defines which color to eject first based on game requirements
     */
    public enum GamePattern {
        GREEN_FIRST, // Green -> Purple -> Purple
        GREEN_SECOND, // Purple -> Green -> Purple
        GREEN_THIRD,
        NULL // Purple -> Purple -> Green
    }

    // Hardware components
    private final DcMotorEx motor;
    private final ColorSensor intakeColor;
    private final DcMotorEx shooterMotor;
    private final Servo feederServo;
    private final ColorSensor shooterColor;

    // State tracking variables
    private int zeroCount = 0;
    private double accum = 0.0;
    private int targetCounts = 0;
    private int intakeIndex = 0;
    private boolean isAtMid = false; // True when at 60° mid-position (eject position)

    // Ball storage (3 slots)
    private final Ball[] slots = { Ball.EMPTY, Ball.EMPTY, Ball.EMPTY };

    // Game pattern for ejection strategy
    private GamePattern pattern = GamePattern.GREEN_FIRST;

    /**
     * Constructor - Initializes the spindexer with motor and color sensor.
     *
     * @param motor       The motor controlling the indexer rotation
     * @param intakeColor Color sensor at the intake position
     */
    /**
     * Constructor - Initializes the spindexer with motors and sensors.
     *
     * @param motor        The motor controlling the indexer rotation
     * @param intakeColor  Color sensor at the intake position
     * @param shooterMotor Motor for the flywheel shooter
     * @param feederServo  Servo to push balls into the shooter
     * @param shooterColor Color sensor at the shooter position
     */
    private final LinearOpMode opMode;

    /**
     * Constructor - Initializes the spindexer with motors and sensors.
     *
     * @param opMode       The OpMode instance (required for loop checks)
     * @param motor        The motor controlling the indexer rotation
     * @param intakeColor  Color sensor at the intake position
     * @param shooterMotor Motor for the flywheel shooter
     * @param feederServo  Servo to push balls into the shooter
     * @param shooterColor Color sensor at the shooter position
     */
    public Spindexer(LinearOpMode opMode, DcMotorEx motor, ColorSensor intakeColor, DcMotorEx shooterMotor,
            Servo feederServo, ColorSensor shooterColor) {
        this.opMode = opMode;
        this.motor = motor;
        this.intakeColor = intakeColor;
        this.shooterMotor = shooterMotor;
        this.feederServo = feederServo;
        this.shooterColor = shooterColor;

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

        // Configure Shooter Motor
        this.shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            this.shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);
        } catch (Exception ignored) {
        }

        // Configure Feeder Servo
        this.feederServo.setPosition(FEEDER_IDLE);
    }

    // ... (Lines 144-558 remain unchanged, but I must preserve imports. Since I'm
    // replacing from line 106, I am safe regarding imports at top)

    // Wait, replacing a huge chunk might be risky if I don't provide the whole
    // chunk.
    // I should only replace the Constructor and the opModeIsActive method.
    // But they are far apart. I should use `multi_replace_file_content` instead.
    // I will use `replace_file_content` individually or switch tool.
    // I'll cancel this tool call and use multi_replace.

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
        // Eject all green balls
        Integer greenIdx;
        while ((greenIdx = findFirst(Ball.GREEN)) != null) {
            ejectSlot(greenIdx, telemetry);
        }

        // Eject all purple balls
        Integer purpleIdx;
        while ((purpleIdx = findFirst(Ball.PURPLE)) != null) {
            ejectSlot(purpleIdx, telemetry);
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
     * Returns the current game pattern for ejection.
     */
    public GamePattern getGamePattern() {
        return pattern;
    }

    /**
     * Sets the game pattern for ejection strategy.
     *
     * @param pattern The desired ejection pattern
     */
    public void setGamePattern(GamePattern pattern) {
        this.pattern = pattern;
    }

    /**
     * Checks if the indexer is at a mid-position (60° between intake stops).
     * This is the ejection position.
     *
     * @return True if at eject position, false if at intake stop
     */
    public boolean isAtMid() {
        return isAtMid;
    }

    /**
     * Checks if all three slots are filled with balls.
     *
     * @return True if all slots contain PURPLE or GREEN balls
     */
    public boolean hasThreeBalls() {
        for (Ball ball : slots) {
            if (ball == Ball.EMPTY) {
                return false;
            }
        }
        return true;
    }

    /**
     * Prepares for the first ejection according to the game pattern.
     * Aligns the indexer and moves +60° to the eject position.
     *
     * @param telemetry Telemetry object for debug output
     * @return True if successfully aligned and positioned, false otherwise
     */
    public boolean prepareFirstEjectByPattern(Telemetry telemetry) {
        if (!hasThreeBalls()) {
            return false; // Cannot prepare if we don't have 3 balls
        }

        // Determine which color to eject first based on pattern
        // GREEN_FIRST starts with GREEN, others start with PURPLE
        Ball targetColor = (pattern == GamePattern.GREEN_FIRST) ? Ball.GREEN : Ball.PURPLE;

        // Find the first ball of target color
        Integer targetSlot = findFirst(targetColor);
        if (targetSlot == null) {
            // Pattern requires a color we don't have, try the other color
            targetColor = (targetColor == Ball.GREEN) ? Ball.PURPLE : Ball.GREEN;
            targetSlot = findFirst(targetColor);
            if (targetSlot == null) {
                return false; // No balls to eject
            }
        }

        // Align so targetSlot will be at eject position after +60°
        int deltaSlots = mod3(targetSlot - 1 - intakeIndex);
        for (int k = 0; k < deltaSlots; k++) {
            move120NoEject(telemetry);
            intakeIndex = mod3(intakeIndex + 1);
        }

        // Move +60° to eject position
        int mid = stepForward60();
        goTo(mid, telemetry);
        isAtMid = true;

        return true;
    }

    /**
     * Ejects all balls according to the game pattern.
     * Follows the priority order defined by the current GamePattern.
     *
     * @param telemetry Telemetry object for debug output
     */
    public void ejectAllByPattern(Telemetry telemetry) {
        startShooter();
        try {
            // Wait briefly for shooter to spin up if needed
            Thread.sleep(500);
        } catch (InterruptedException ignored) {
        }

        switch (pattern) {
            case GREEN_FIRST:
                ejectAllGreenThenPurple(telemetry);
                break;
            case GREEN_SECOND:
                ejectGreenSecond(telemetry);
                break;
            case GREEN_THIRD:
                ejectAllPurpleThenGreen(telemetry);
                break;
        }
        stopShooter();
        isAtMid = false; // After full ejection, we end at an intake stop
    }

    /**
     * Ejects balls: One PURPLE, then all GREENs, then all remaining PURPLEs.
     *
     * @param telemetry Telemetry object for debug output
     */
    private void ejectGreenSecond(Telemetry telemetry) {
        // 1. Eject the first purple ball
        Integer firstPurple = findFirst(Ball.PURPLE);
        if (firstPurple != null) {
            ejectSlot(firstPurple, telemetry);
        }

        // 2. Eject all green balls
        Integer greenIdx;
        while ((greenIdx = findFirst(Ball.GREEN)) != null) {
            ejectSlot(greenIdx, telemetry);
        }

        // 3. Eject all remaining purple balls
        Integer purpleIdx;
        while ((purpleIdx = findFirst(Ball.PURPLE)) != null) {
            ejectSlot(purpleIdx, telemetry);
        }
    }

    /**
     * Ejects all PURPLE balls first, then all GREEN balls.
     *
     * @param telemetry Telemetry object for debug output
     */
    private void ejectAllPurpleThenGreen(Telemetry telemetry) {
        // Eject purple balls first
        for (int i = 0; i < 3; i++) {
            if (slots[i] == Ball.PURPLE) {
                ejectSlot(i, telemetry);
            }
        }

        // Then eject all green balls
        Integer greenIdx;
        while ((greenIdx = findFirst(Ball.GREEN)) != null) {
            ejectSlot(greenIdx, telemetry);
        }
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

        // VERIFICATION: Check color at shooter before ejecting
        Ball expected = slots[slotIndex];
        Ball actual = readColorAtShooter();

        // Only shoot if the actual color matches the expectation (and is not EMPTY)
        // If mismatch, update the slot to the actual color and SKIP shooting
        if (actual == expected && actual != Ball.EMPTY) {
            // Actuate feeder servo to push ball into shooter
            feederServo.setPosition(FEEDER_EJECT);
            try {
                Thread.sleep(FEEDER_SLEEP_MS);
            } catch (InterruptedException ignored) {
            }

            feederServo.setPosition(FEEDER_IDLE);
            try {
                Thread.sleep(FEEDER_SLEEP_MS); // Wait for servo to return
            } catch (InterruptedException ignored) {
            }

            // Clear the slot only after successful ejection
            slots[slotIndex] = Ball.EMPTY;
        } else {
            // Mismatch detected! Update internal state to match reality
            slots[slotIndex] = actual;
            if (telemetry != null) {
                telemetry.addData("Verification", "Mismatch at slot %d! Expected %s, Found %s", slotIndex, expected,
                        actual);
                telemetry.update();
            }
        }

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
        isAtMid = true;
        goTo(mid, telemetry);
        int stop = stepForward60();
        isAtMid = false;
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
        return readColor(intakeColor);
    }

    /**
     * Reads and identifies the ball color at the shooter position.
     *
     * @return Ball color (EMPTY, PURPLE, or GREEN)
     */
    private Ball readColorAtShooter() {
        return readColor(shooterColor);
    }

    /**
     * Helper to read color from a specific sensor.
     */
    private Ball readColor(ColorSensor sensor) {
        if (sensor.alpha() <= PRESENCE_ALPHA_THRESHOLD)
            return Ball.EMPTY;

        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        // Color classification logic
        boolean isPurple = (b > g + 5) && (r > g + 5);
        boolean isGreen = (g > r + 5) && (g > b + 5);

        if (isGreen && !isPurple)
            return Ball.GREEN;
        if (isPurple && !isGreen)
            return Ball.PURPLE;

        // Fallback: choose dominant color channel
        if (g >= r && g >= b)
            return Ball.GREEN;
        return Ball.PURPLE;
    }

    /**
     * Checks if the OpMode is still active.
     * WARNING: This implementation always returns true.
     * In a real OpMode, inject a reference to LinearOpMode and check
     * opModeIsActive().
     *
     * @return Always true (placeholder implementation)
     */
    private boolean opModeIsActive() {
        return opMode.opModeIsActive();
    }

    /**
     * Spins up the shooter motor to target velocity.
     */
    public void startShooter() {
        shooterMotor.setVelocity(SHOOTER_VELOCITY);
    }

    /**
     * Stops the shooter motor.
     */
    public void stopShooter() {
        shooterMotor.setVelocity(0);
    }

    public double getShooterVelocity() {
        return shooterMotor.getVelocity();
    }
}
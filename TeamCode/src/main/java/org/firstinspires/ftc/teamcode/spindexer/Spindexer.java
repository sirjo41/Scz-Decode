package org.firstinspires.ftc.teamcode.spindexer;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    public static final double TICKS_PER_SLOT = 473;// TODO: Tune number of slots (currently 3)

    // PID coefficients for position control
    public static final PIDFCoefficients POS_PIDF = new PIDFCoefficients(
            12, // TODO: Tune P coefficient
            0, // TODO: Tune I coefficient
            0.15, // TODO: Tune D coefficient
            0.0); // TODO: Tune F coefficient

    // Motor control parameters
    private static final int TARGET_TOL = 1; // TODO: Tune position tolerance
    private static final double MAX_POWER = 1; // TODO: Tune max motor power

    // Shooter constants
    public static final double TARGET_SHOOTER_RPM = 1800; // TODO: Tune target shooter velocity
    private static final double SHOOTER_VELOCITY_TOLERANCE = 200; // RPM tolerance for "ready" state

    // Shooter PID coefficients
    // Shooter PID coefficients
    public static final double SHOOTER_P = 0; // Tune: Start with 10% of F
    public static final double SHOOTER_I = 0.0;
    public static final double SHOOTER_D = 0.0;
    public static final double SHOOTER_F = 24; // Tune: 32767 / MaxTicksPerSec (approx 2700 for 6000RPM motor)

    // Servo positions
    private static final double FEEDER_IDLE = 1.0;
    private static final double FEEDER_FEEDING = 0.55;

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

    /**
     * Spindexer mode enumeration.
     */
    public enum SpindexerMode {
        INTAKING,
        SHOOTING
    }

    // Hardware components
    private final DcMotorEx motor;
    private final NormalizedColorSensor intakeSensor;
    private final LinearOpMode opMode;
    private final Servo feederServo;
    private final DcMotorEx shooterMotor;

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

    // Spindexer mode
    private SpindexerMode mode = SpindexerMode.INTAKING;

    // Shooting state
    public enum ShootingState {
        SEARCHING,
        MOVING,
        READY_TO_SHOOT,
        FEEDING,
        SHOOTING_ACTION,
        COOLDOWN
    }

    private ShootingState shootingState = ShootingState.SEARCHING;
    private long shootTimer = 0;

    private int ballsShotCount = 0;

    /**
     * Checks if the spindexer is full (all 3 slots have balls).
     */
    public boolean isFull() {
        for (SlotColor slot : slots) {
            if (slot == SlotColor.EMPTY) {
                return false;
            }
        }
        return true;
    }

    /**
     * Finds the relative index of the next slot containing the specified color.
     * Searches starting from the current slot index.
     * Returns -1 if not found.
     */
    public int getNextSlotWithColor(SlotColor color) {
        // Start check at currentSlotIndex, effectively the "0" point relative to
        // shooter if aligned
        // We need to check all 3 slots
        for (int i = 0; i < slots.length; i++) {
            // (current + i) % 3 loops 0 -> 1 -> 2 relative to current
            int checkIndex = (currentSlotIndex + i) % slots.length;
            if (slots[checkIndex] == color) {
                return i; // Return the relative offset (0, 1, or 2)
            }
        }
        return -1;
    }

    public int getBallsShotCount() {
        return ballsShotCount;
    }

    public void resetShotCount() {
        ballsShotCount = 0;
    }

    /**
     * Constructor - Initializes the spindexer with motor and intake system.
     */
    public Spindexer(LinearOpMode opMode, DcMotorEx motor,
            NormalizedColorSensor intakeSensor, Servo feederServo, DcMotorEx shooterMotor) {
        this.opMode = opMode;
        this.motor = motor;
        this.intakeSensor = intakeSensor;
        this.feederServo = feederServo;
        this.shooterMotor = shooterMotor;

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

        // Initialize feeder servo to idle position
        feederServo.setPosition(FEEDER_IDLE);

        // Initialize shooter motor
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Apply PIDF coefficients for velocity control
        // Note: F (Feedforward) is critical for velocity! F = 32767 / MaxTicksPerSec
        PIDFCoefficients currentShooterPIDF = new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, currentShooterPIDF);
    }

    /**
     * Move left (counter-clockwise) by one slot.
     */
    public void moveLeft(Telemetry telemetry) {
        feederServo.setPosition(FEEDER_IDLE);
        accum -= TICKS_PER_SLOT;
        int target = (int) Math.rint(zeroCount + accum);
        setTarget(target);
    }

    /**
     * Move right (clockwise) by one slot.
     */
    public void moveRight(Telemetry telemetry) {
        feederServo.setPosition(FEEDER_IDLE);
        accum += TICKS_PER_SLOT;
        int target = (int) Math.rint(zeroCount + accum);
        setTarget(target);
    }

    /**
     * Move left (counter-clockwise) by half a slot (for shooting mode).
     */
    public void moveLeftHalf(Telemetry telemetry) {
        feederServo.setPosition(FEEDER_IDLE);
        accum -= TICKS_PER_SLOT / 2.0;
        int target = (int) Math.rint(zeroCount + accum);
        setTarget(target);
    }

    /**
     * Move right (clockwise) by half a slot (for shooting mode).
     */
    public void moveRightHalf(Telemetry telemetry) {
        feederServo.setPosition(FEEDER_IDLE);
        accum += TICKS_PER_SLOT / 2.0;
        int target = (int) Math.rint(zeroCount + accum);
        setTarget(target);
    }

    /**
     * Gets the current spindexer mode.
     */
    public SpindexerMode getMode() {
        return mode;
    }

    /**
     * Sets the spindexer mode.
     */
    public void setMode(SpindexerMode mode) {
        this.mode = mode;
        if (mode == SpindexerMode.SHOOTING) {
            shootingState = ShootingState.SEARCHING;
            ballsShotCount = 0; // Reset count on manual entry? Maybe.
        }
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
        feederServo.setPosition(FEEDER_IDLE);
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
        feederServo.setPosition(FEEDER_IDLE);
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
        NormalizedRGBA colors = intakeSensor.getNormalizedColors();
        // TODO: Tune these color thresholds based on your sensor and lighting
        if (colors.green < 0.0030 && colors.blue < 0.0030) {
            return SlotColor.EMPTY;
        } else if (colors.green > colors.blue) {
            return SlotColor.GREEN;
        } else if (colors.blue > colors.green) { // colors.red > colors.green &&
            return SlotColor.PURPLE;
        } else {
            return SlotColor.EMPTY;
        }
    }

    /**
     * Updates intake system - checks color sensor and stores color if object
     * detected.
     * Call this periodically in your main loop.
     */
    public void updateIntake() {
        if (mode == SpindexerMode.SHOOTING) {
            updateAutoShoot();
            return;
        }

        // Busy check: don't look for new balls while moving to the next slot
        if (motor.isBusy()) {
            return;
        }

        SlotColor detectedColor = detectColor();

        // 1. Determine if *currently* detecting an object
        boolean objectCurrentlyDetected = (detectedColor == SlotColor.PURPLE || detectedColor == SlotColor.GREEN);

        // 2. Rising edge check: Only act if we see an object NOW, but didn't see one
        // BEFORE
        if (objectCurrentlyDetected && !lastColorDetected) {
            // Store color in current slot and advance to next slot
            if (currentSlotIndex < slots.length) {
                slots[currentSlotIndex] = detectedColor;
                currentSlotIndex = (currentSlotIndex + 1) % slots.length;

                // Prioritize checking if full to switch mode
                if (isFull()) {
                    mode = SpindexerMode.SHOOTING;
                    moveRightHalf(this.opMode.telemetry); // Enter shooting position 1.5

                    // Physical Correction: When we stop at intake (Slot 2) and move +0.5,
                    // Slot 1 ends up at the shooter (180 deg away).
                    // So we must shift our index reference by +1 to match physical reality.
                    currentSlotIndex = (currentSlotIndex + 1) % slots.length;

                    shootingState = ShootingState.SEARCHING; // Start sorting
                } else {
                    // Not full yet, just move to next slot
                    moveRight(this.opMode.telemetry);
                }
            }
        }

        // 3. Update state for next loop to enable edge detection
        lastColorDetected = objectCurrentlyDetected;
    }

    /**
     * Handles the sorting and shooting sequence.
     */
    public void updateAutoShoot() {
        switch (shootingState) {
            case SEARCHING:
                if (ballsShotCount >= 3) {
                    // Done, exit
                    stopShooter();
                    moveRightHalf(this.opMode.telemetry); // Exit 1.5 back to intake
                    mode = SpindexerMode.INTAKING;
                    ballsShotCount = 0;
                    clearSlots();
                    return;
                }

                // Determine target color
                SlotColor targetColor = SlotColor.PURPLE;
                if (pattern == GamePattern.GREEN_FIRST) {
                    targetColor = (ballsShotCount == 0) ? SlotColor.GREEN : SlotColor.PURPLE;
                } else if (pattern == GamePattern.GREEN_SECOND) {
                    targetColor = (ballsShotCount == 1) ? SlotColor.GREEN : SlotColor.PURPLE;
                } else if (pattern == GamePattern.GREEN_THIRD) {
                    targetColor = (ballsShotCount == 2) ? SlotColor.GREEN : SlotColor.PURPLE;
                }

                int relIndex = getNextSlotWithColor(targetColor);
                if (relIndex == -1) {
                    // Fallback: search for ANY non-empty
                    relIndex = getNextSlotWithColor(SlotColor.PURPLE);
                    if (relIndex == -1)
                        relIndex = getNextSlotWithColor(SlotColor.GREEN);
                    if (relIndex == -1) {
                        // No balls left?
                        ballsShotCount = 3; // Force exit
                        return;
                    }
                }

                // Move to that slot
                if (relIndex > 0) {
                    // Shortest path optimization:
                    // If target is 2 slots away (forward), it's faster to go 1 slot backward.
                    // relIndex is always 0, 1, or 2.
                    int moveSteps = relIndex;
                    if (moveSteps > 1) {
                        moveSteps = -1; // Move backward 1 slot instead of forward 2
                    }

                    accum += moveSteps * TICKS_PER_SLOT;
                    int t = (int) Math.rint(zeroCount + accum);
                    setTarget(t); // Non-blocking set

                    // Verify update of current index logic
                    // We moved 'relIndex' slots forward (logically).
                    currentSlotIndex = (currentSlotIndex + relIndex) % slots.length;
                }
                shootingState = ShootingState.MOVING;
                break;

            case MOVING:
                if (!motor.isBusy()) {
                    shootingState = ShootingState.READY_TO_SHOOT;
                }
                break;

            case READY_TO_SHOOT:
                spinUpShooter();
                if (isShooterReady()) {
                    feederServo.setPosition(FEEDER_FEEDING);
                    shootTimer = System.currentTimeMillis();
                    shootingState = ShootingState.FEEDING;
                }
                break;

            case FEEDING:
                spinUpShooter(); // Keep spinning
                if (System.currentTimeMillis() - shootTimer > 500) { // Wait 500ms for feed
                    feederServo.setPosition(FEEDER_IDLE);
                    shootingState = ShootingState.SHOOTING_ACTION;
                }
                break;

            case SHOOTING_ACTION:
                // Update state
                slots[currentSlotIndex] = SlotColor.EMPTY; // Ball shot
                ballsShotCount++;
                shootingState = ShootingState.COOLDOWN;
                shootTimer = System.currentTimeMillis();

            case COOLDOWN:
                if (System.currentTimeMillis() - shootTimer > 300) { // Wait 300ms before moving
                    shootingState = ShootingState.SEARCHING;
                }
                break;
        }
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
        stopShooter();
    }

    // ==================== SHOOTER METHODS ====================

    /**
     * Spins up the shooter motor to target velocity.
     */
    public void spinUpShooter() {
        // Convert RPM to Ticks Per Second
        // Velocity = (RPM / 60) * CPR
        double targetTicksPerSecond = (TARGET_SHOOTER_RPM / 60.0) * CPR_MOTOR;
        shooterMotor.setVelocity(targetTicksPerSecond);
    }

    /**
     * Stops the shooter motor.
     */
    public void stopShooter() {
        shooterMotor.setVelocity(0);
        feederServo.setPosition(FEEDER_IDLE);
    }

    /**
     * Gets the current shooter velocity in RPM.
     */
    public double getShooterRPM() {
        double ticksPerSecond = shooterMotor.getVelocity();
        // RPM = (TicksPerSec / CPR) * 60
        return (ticksPerSecond / CPR_MOTOR) * 60.0;
    }

    /**
     * Checks if shooter is at target velocity and ready to shoot.
     */
    public boolean isShooterReady() {
        double currentVelocity = getShooterRPM();
        return (Math.abs(currentVelocity) >= TARGET_SHOOTER_RPM);
    }

    /**
     * Feeds a ball into the shooter when ready and in SHOOTING mode.
     * Returns true if fed, false if shooter not ready or not in shooting mode.
     */
    public boolean shoot() {
        // Only shoot if in SHOOTING mode AND shooter is ready
        if (mode == SpindexerMode.SHOOTING) {
            feederServo.setPosition(FEEDER_FEEDING);
            return true;
        }
        return false;
    }

    /**
     * Resets feeder to idle position.
     */
    public void retractFeeder() {
        feederServo.setPosition(FEEDER_IDLE);
    }

    /**
     * Adds shooter telemetry data.
     */
    public void addShooterTelemetry(Telemetry telemetry) {
        telemetry.addLine("=== Shooter State ===");
        telemetry.addData("Target RPM", TARGET_SHOOTER_RPM);
        telemetry.addData("Current RPM", String.format("%.0f", getShooterRPM()));
        telemetry.addData("Shooter Ready", isShooterReady() ? "YES" : "NO");
        telemetry.addData("Feeder Position", String.format("%.2f", feederServo.getPosition()));
    }

    public void addSensorTelemetry(Telemetry telemetry) {
        NormalizedRGBA colors = intakeSensor.getNormalizedColors();
        telemetry.addLine("=== Sensor State ===");
        telemetry.addData("RED", colors.red);
        telemetry.addData("BLUE", colors.blue);
        telemetry.addData("Green", colors.green);
    }
}

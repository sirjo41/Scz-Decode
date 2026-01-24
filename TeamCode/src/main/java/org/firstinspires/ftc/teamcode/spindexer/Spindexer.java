package org.firstinspires.ftc.teamcode.spindexer;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

/**
 * Spindexer - Simplified rotary indexer for left/right movement.
 */
@Configurable
public class Spindexer {

    // Motor and encoder constants
    private static final double CPR_MOTOR = 28; // TODO: Verify motor encoder CPR

    // Rotation constants
    public static final double TICKS_PER_SLOT = 473;// TODO: Tune number of slots (currently 3)

    // PID coefficients for position control
    public static final PIDFCoefficients POS_PIDF = new PIDFCoefficients(
            12, // TODO: Tune P coefficient
            0,
            0.15,
            0.0);

    // Motor control parameters
    private static final int TARGET_TOL = 1;
    private static final double MAX_POWER = 1;

    // Shooter constants
    public static final double TARGET_SHOOTER_RPM = 1500; // TODO: Tune target shooter velocity
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
        WAITING_FOR_TRIGGER,
        FEEDING,
        SHOOTING_ACTION,
        COOLDOWN
    }

    private ShootingState shootingState = ShootingState.SEARCHING;
    private long shootTimer = 0;
    private boolean shootRequested = false;
    private boolean semiAutoMode = true; // Default to TeleOp behavior (wait for trigger)

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
    public Spindexer(OpMode opMode, DcMotorEx motor,
                     NormalizedColorSensor intakeSensor, Servo feederServo, DcMotorEx shooterMotor) {
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
        Arrays.fill(slots, SlotColor.EMPTY);

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
    public void moveLeft() {
        feederServo.setPosition(FEEDER_IDLE);
        accum -= TICKS_PER_SLOT;
        int target = (int) Math.rint(zeroCount + accum);
        setTarget(target);
        currentSlotIndex = (currentSlotIndex - 1 + slots.length) % slots.length;
    }

    /**
     * Move right (clockwise) by one slot.
     */
    public void moveRight() {
        feederServo.setPosition(FEEDER_IDLE);
        accum += TICKS_PER_SLOT;
        int target = (int) Math.rint(zeroCount + accum);
        setTarget(target);
        currentSlotIndex = (currentSlotIndex + 1) % slots.length;
    }

    /**
     * Move left (counter-clockwise) by half a slot (for shooting mode).
     */
    public void moveLeftHalf() {
        feederServo.setPosition(FEEDER_IDLE);
        accum -= TICKS_PER_SLOT / 2.0;
        int target = (int) Math.rint(zeroCount + accum);
        setTarget(target);
    }

    /**
     * Move right (clockwise) by half a slot (for shooting mode).
     */
    public void moveRightHalf() {
        feederServo.setPosition(FEEDER_IDLE);
        accum += TICKS_PER_SLOT / 2.0;
        int target = (int) Math.rint(zeroCount + accum);
        setTarget(target);
        // Half move usually implies transitioning to shooting, index shift handled
        // elsewhere or logic specific
        // For consistency, we might shift index if we consider the "active" slot
        // changed?
        // Let's assume Half Move pushes slot X out of intake view.
        // currentSlotIndex = (currentSlotIndex + 1) % slots.length; // Optional? Stick
        // to existing logic for now.
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
            shootRequested = false;
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
     * Checks if the OpMode is still active.
     */
    private boolean opModeIsActive() {
        return true; // Simple workaround since OpMode doesn't have isActive() like LinearOpMode.
        // Logic relying on this should rely on external loop control or exception
        // handling if OpMode stops.
        // Or cast if known type, but for now true is safer than crashing on cast
        // failure.
    }

    /**
     * Detects color from sensor RGB values.
     */
    private SlotColor detectColor() {
        NormalizedRGBA colors = intakeSensor.getNormalizedColors();
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

        // Monitor current slot content continuously
        SlotColor detectedColor = detectColor();
        if (detectedColor != SlotColor.EMPTY) {
            slots[currentSlotIndex] = detectedColor;
        }

        // Logic for "New Ball Arrival" in TeleOp
        // If we are *supposed* to be catching new balls, and we see one:
        // 1. Keep it.
        // 2. Move to next slot.

        // Rising edge check for MOVE Trigger
        boolean objectCurrentlyDetected = (detectedColor == SlotColor.PURPLE || detectedColor == SlotColor.GREEN);

        if (objectCurrentlyDetected && !lastColorDetected) {
            // We found a NEW ball.
            // Move to next slot to accept more.
            // Check if we are physically capable of moving (busy check handled at top)
            if (!isFull()) {
                moveRight();
                // Note: moveRight updates currentSlotIndex automatically now.
            } else {
                // Full? Switch to shooting?
                mode = SpindexerMode.SHOOTING;
                moveRightHalf();
                shootingState = ShootingState.SEARCHING;
            }
        }

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
                    moveRightHalf(); // Exit 1.5 back to intake
                    mode = SpindexerMode.INTAKING;
                    ballsShotCount = 0;
                    clearSlots();
                    return;
                }
                int relIndex = -1;

                    SlotColor targetColor = getSlotColor();

                    relIndex = getNextSlotWithColor(targetColor);
                    if (relIndex == -1) {
                        // Fallback: search for ANY non-empty
                        relIndex = getNextSlotWithColor(SlotColor.PURPLE);
                        if (relIndex == -1)
                            relIndex = getNextSlotWithColor(SlotColor.GREEN);
                    }
                if (relIndex == -1) {
                    // No balls left?
                    ballsShotCount = 3; // Force exit
                    return;
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
                if (isShooterReady()) {
                    shootingState = ShootingState.WAITING_FOR_TRIGGER;
                }
                break;

            case WAITING_FOR_TRIGGER:
                // Wait here until user presses Y (shootRequested) OR we are in fully auto mode
                if ((shootRequested || !semiAutoMode) && isShooterReady()) {
                    feederServo.setPosition(FEEDER_FEEDING);
                    shootTimer = System.currentTimeMillis();
                    shootingState = ShootingState.FEEDING;
                    shootRequested = false; // Reset trigger
                }
                break;

            case FEEDING:
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

    private @NonNull SlotColor getSlotColor() {
        SlotColor targetColor = SlotColor.PURPLE;
        if (pattern == GamePattern.GREEN_FIRST) {
            targetColor = (ballsShotCount == 0) ? SlotColor.GREEN : SlotColor.PURPLE;
        } else if (pattern == GamePattern.GREEN_SECOND) {
            targetColor = (ballsShotCount == 1) ? SlotColor.GREEN : SlotColor.PURPLE;
        } else if (pattern == GamePattern.GREEN_THIRD) {
            targetColor = (ballsShotCount == 2) ? SlotColor.GREEN : SlotColor.PURPLE;
        }
        return targetColor;
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
        Arrays.fill(slots, SlotColor.EMPTY);
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
        double targetTicksPerSecond = TARGET_SHOOTER_RPM;
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
        return (ticksPerSecond );
    }

    /**
     * Checks if shooter is at target velocity and ready to shoot.
     */
    public boolean isShooterReady() {
        double currentVelocity = getShooterRPM();
        return (Math.abs(currentVelocity) >= TARGET_SHOOTER_RPM);
    }

    /**
     * Sets the shoot request flag. The state machine will handle the actual
     * feeding.
     * Returns true if request accepted.
     */
    public void shoot() {
        // Only accept if in SHOOTING mode
        if (mode == SpindexerMode.SHOOTING) {
            shootRequested = true;
        }
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
    @SuppressLint("DefaultLocale")
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

    /**
     * Scans all slots to detect pre-loaded balls.
     * Rotates one full revolution.
     */
    public void scanSlots() {
        for (int i = 0; i < 3; i++) {
            // Wait a bit for sensor (blocking in auto init/start typically ok-ish if brief)
            long start = System.currentTimeMillis();
            while (System.currentTimeMillis() - start < 200) {
                // Sample color
                SlotColor c = detectColor();
                if (c != SlotColor.EMPTY)
                    slots[currentSlotIndex] = c;
            }
            // Move to next
            moveRight();
            // Wait for move?
            while (motor.isBusy() && opModeIsActive()) {
                // idle
            }
        }
    }

    public void setSemiAutoMode(boolean enabled) {
        this.semiAutoMode = enabled;
    }
}

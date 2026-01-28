package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.spindexer.Shooter;

import java.util.Arrays;

@Configurable
public class SpindexerAuto {

    /* ================= CONSTANTS ================= */

    public static final double TICKS_PER_SLOT = 473;
    private static final int TARGET_TOL = 1;
    private static final double MAX_POWER = 0.8;

    public static final PIDFCoefficients POS_PIDF = new PIDFCoefficients(0.2, 0.001, 0.05, 0.0032);

    /* ================= ENUMS ================= */

    public enum GamePattern {
        GREEN_FIRST,
        GREEN_SECOND,
        GREEN_THIRD
    }

    public enum SlotColor {
        EMPTY,
        PURPLE,
        GREEN
    }

    public enum SpindexerMode {
        INTAKING,
        SHOOTING
    }

    private enum ShootingState {
        SEARCHING,
        MOVING,
        FEEDING,
        POST_SHOT_DELAY
    }

    /* ================= HARDWARE ================= */

    private final DcMotorEx motor;
    private final NormalizedColorSensor intakeSensor;
    private final Shooter shooter;

    /* ================= STATE ================= */

    private final SlotColor[] slots = new SlotColor[3];
    private int currentSlotIndex = 0;

    private double accum = 0;
    private int zeroCount;
    private int targetCounts;

    private GamePattern pattern = GamePattern.GREEN_FIRST;
    private SpindexerMode mode = SpindexerMode.INTAKING;
    private ShootingState shootingState = ShootingState.SEARCHING;

    private int ballsShot = 0;
    private long timer = 0;

    private boolean lastObjectDetected = false;

    /* ================= CONSTRUCTOR ================= */

    public SpindexerAuto(OpMode opMode,
            DcMotorEx motor,
            NormalizedColorSensor intakeSensor,
            Shooter shooter) {

        this.motor = motor;
        this.intakeSensor = intakeSensor;
        this.shooter = shooter;

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setTargetPositionTolerance(TARGET_TOL);

        try {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, POS_PIDF);
        } catch (Exception ignored) {
        }

        zeroCount = motor.getCurrentPosition();
        targetCounts = zeroCount;
        motor.setTargetPosition(targetCounts);
        motor.setPower(MAX_POWER);

        Arrays.fill(slots, SlotColor.EMPTY);
    }

    /* ================= PUBLIC API ================= */

    public void setGamePattern(GamePattern pattern) {
        this.pattern = pattern;
    }

    public void setMode(SpindexerMode mode) {
        this.mode = mode;
    }

    public void spinUpShooter() {
        shooter.spinUpShooter();
    }

    public void stopShooter() {
        shooter.stopShooter();
    }

    public SpindexerMode getMode() {
        return mode;
    }

    public int getBallsShotCount() {
        return ballsShot;
    }

    public SlotColor getSlotColor(int i) {
        return slots[i];
    }

    /* ================= INTAKE ================= */

    public void updateIntake() {
        if (mode != SpindexerMode.INTAKING || motor.isBusy())
            return;

        SlotColor detected = detectColor();
        boolean objectDetected = detected != SlotColor.EMPTY;

        if (objectDetected) {
            slots[currentSlotIndex] = detected;
        }

        if (objectDetected && !lastObjectDetected) {
            if (!isFull()) {
                moveSlots(1);
            } else {
                // Full → transition to shooting
                mode = SpindexerMode.SHOOTING;
                shooter.spinUpShooter();
                shootingState = ShootingState.SEARCHING;
                moveHalfSlotRight();
                currentSlotIndex = (currentSlotIndex - 1 + 3) % 3;
            }
        }

        lastObjectDetected = objectDetected;
    }

    /* ================= SHOOTING ================= */

    public void updateAutoShoot() {
        if (mode != SpindexerMode.SHOOTING)
            return;

        switch (shootingState) {

            case SEARCHING:
                if (ballsShot >= 3) {
                    exitShooting();
                    return;
                }

                SlotColor target = getTargetColor();
                int rel = findNextSlot(target);

                if (rel == -1)
                    rel = findAnyBall();
                if (rel == -1) {
                    exitShooting();
                    return;
                }

                if (rel != 0) {
                    int move = (rel == 2) ? -1 : rel;
                    moveSlots(move);
                }

                shootingState = ShootingState.MOVING;
                break;

            case MOVING:
                if (!motor.isBusy() && shooter.isShooterReady()) {
                    shooter.feed();
                    timer = System.currentTimeMillis();
                    shootingState = ShootingState.FEEDING;
                }
                break;

            case FEEDING:
                if (System.currentTimeMillis() - timer > 200) {
                    shooter.retractFeeder();
                    slots[currentSlotIndex] = SlotColor.EMPTY;
                    ballsShot++;
                    timer = System.currentTimeMillis();
                    shootingState = ShootingState.POST_SHOT_DELAY;
                }
                break;

            case POST_SHOT_DELAY:
                if (System.currentTimeMillis() - timer > 120) {
                    shootingState = ShootingState.SEARCHING;
                }
                break;
        }
    }

    /* ================= MOTION ================= */

    private void moveSlots(int slotsToMove) {
        shooter.retractFeeder();
        accum += slotsToMove * TICKS_PER_SLOT;
        targetCounts = (int) Math.rint(zeroCount + accum);
        motor.setTargetPosition(targetCounts);
        motor.setPower(MAX_POWER);
        currentSlotIndex = (currentSlotIndex + slotsToMove + 3) % 3;
    }

    private void moveHalfSlotRight() {
        shooter.retractFeeder();
        accum += TICKS_PER_SLOT / 2.0;
        targetCounts = (int) Math.rint(zeroCount + accum);
        motor.setTargetPosition(targetCounts);
        motor.setPower(MAX_POWER);
    }

    /* ================= HELPERS ================= */

    private boolean isFull() {
        for (SlotColor c : slots)
            if (c == SlotColor.EMPTY)
                return false;
        return true;
    }

    private int findNextSlot(SlotColor c) {
        for (int i = 0; i < 3; i++) {
            int idx = (currentSlotIndex + i) % 3;
            if (slots[idx] == c)
                return i;
        }
        return -1;
    }

    private int findAnyBall() {
        int r = findNextSlot(SlotColor.GREEN);
        return (r != -1) ? r : findNextSlot(SlotColor.PURPLE);
    }

    private SlotColor getTargetColor() {
        if (pattern == GamePattern.GREEN_FIRST)
            return ballsShot == 0 ? SlotColor.GREEN : SlotColor.PURPLE;
        if (pattern == GamePattern.GREEN_SECOND)
            return ballsShot == 1 ? SlotColor.GREEN : SlotColor.PURPLE;
        return ballsShot == 2 ? SlotColor.GREEN : SlotColor.PURPLE;
    }

    private void exitShooting() {
        shooter.stopShooter();
        moveHalfSlotRight();
        Arrays.fill(slots, SlotColor.EMPTY);
        ballsShot = 0;
        mode = SpindexerMode.INTAKING;
    }

    private SlotColor detectColor() {
        NormalizedRGBA c = intakeSensor.getNormalizedColors();
        if (c.green > 0.05 && c.green > c.blue)
            return SlotColor.GREEN;
        if (c.blue > 0.05 && c.blue > c.green)
            return SlotColor.PURPLE;
        return SlotColor.EMPTY;
    }

    /* ================= TELEMETRY ================= */

    @SuppressLint("DefaultLocale")
    public void addTelemetry(Telemetry t) {
        t.addData("Mode", mode);
        t.addData("SlotIdx", currentSlotIndex);
        t.addData("Slots", Arrays.toString(slots));
        t.addData("BallsShot", ballsShot);
        t.addData("ShooterReady", shooter.isShooterReady());
    }

    /* ================= INIT SCAN ================= */

    public void scanSlots() {
        for (int i = 0; i < 3; i++) {
            long start = System.currentTimeMillis();
            while (System.currentTimeMillis() - start < 200) {
                SlotColor c = detectColor();
                if (c != SlotColor.EMPTY)
                    slots[currentSlotIndex] = c;
            }
            moveSlots(1);
            while (motor.isBusy()) {
            }
        }
    }
}

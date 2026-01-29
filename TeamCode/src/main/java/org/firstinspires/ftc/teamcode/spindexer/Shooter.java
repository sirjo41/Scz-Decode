package org.firstinspires.ftc.teamcode.spindexer;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Shooter - Controls the flywheel shooter and feeder servo.
 */
public class Shooter {

    // Shooter constants
    public static double TARGET_SHOOTER_RPM = 1300; // TODO: Tune target shooter velocity

    // Shooter PID coefficients
    public static final double SHOOTER_P = 55; // Tune: Start with 10% of F
    public static final double SHOOTER_I = 0.0;
    public static final double SHOOTER_D = 0.0;
    public static final double SHOOTER_F = 15; // Tune: 32767 / MaxTicksPerSec (approx 2700 for 6000RPM motor)

    // Servo positions
    private static final double FEEDER_IDLE = 1.0;
    private static final double FEEDER_FEEDING = 0.55;

    // Hardware components
    private final DcMotorEx shooterMotor;
    private final Servo feederServo;

    // State tracking
    private boolean shooterActive = false;

    /**
     * Constructor - Initializes the shooter with motor and servo.
     */
    public Shooter(DcMotorEx shooterMotor, Servo feederServo) {
        this.shooterMotor = shooterMotor;
        this.feederServo = feederServo;

        // Initialize shooter motor
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Apply PIDF coefficients for velocity control
        // Note: F (Feedforward) is critical for velocity! F = 32767 / MaxTicksPerSec
        PIDFCoefficients shooterPIDF = new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        // Initialize feeder servo to idle position
        feederServo.setPosition(FEEDER_IDLE);
    }

    /**
     * Spins up the shooter motor to target velocity.
     */
    public void spinUpShooter() {
        shooterActive = true;
    }

    /**
     * Stops the shooter motor.
     */
    public void stopShooter() {
        shooterActive = false;
        shooterMotor.setVelocity(0);
        feederServo.setPosition(FEEDER_IDLE);
    }

    /**
     * Call this every loop to maintain shooter velocity.
     * This ensures the shooter keeps spinning even when trigger is released.
     */
    public void updateShooter() {
        if (shooterActive) {
            shooterMotor.setVelocity(TARGET_SHOOTER_RPM);
        }
    }

    /**
     * Gets the current shooter velocity in RPM.
     */
    public double getShooterRPM() {
        // RPM = (TicksPerSec / CPR) * 60
        return (shooterMotor.getVelocity());
    }

    /**
     * Checks if shooter is at target velocity and ready to shoot.
     */
    public boolean isShooterReady() {
        double currentVelocity = getShooterRPM();
        // RPM tolerance for "ready" state
        double SHOOTER_VELOCITY_TOLERANCE = 100;
        return (Math.abs(currentVelocity - TARGET_SHOOTER_RPM) < SHOOTER_VELOCITY_TOLERANCE);
    }

    /**
     * Activates the feeder to push ball into shooter.
     */
    public void feed() {
        feederServo.setPosition(FEEDER_FEEDING);
    }

    /**
     * Resets feeder to idle position.
     */
    public void retractFeeder() {
        feederServo.setPosition(FEEDER_IDLE);
    }

    /**
     * Sets the feeder servo position directly.
     */
    public void setFeederPosition(double position) {
        feederServo.setPosition(position);
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
}

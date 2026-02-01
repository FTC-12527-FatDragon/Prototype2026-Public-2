package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem handling the Intake mechanism.
 * Controls the intake motor for collecting and ejecting game elements.
 */
public class Intake extends SubsystemBase {
    public final DcMotorEx intakeMotor;
    public final TelemetryPacket packet = new TelemetryPacket();

    public static boolean isRunning = false;
    public static boolean isShooting = false;
    public static boolean isFullPower = false;
    public static boolean isReversed = false;
    public static boolean isFastShooting = false; // For FAST shooting mode in auto
    public static boolean isFastIntaking = false;
    
    // Emergency disable flag (controlled by gamepad2)
    private boolean disabled = false;

    /**
     * Constructor for Intake.
     * Initializes the intake motor and sets default state.
     *
     * @param hardwareMap The hardware map.
     */
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, IntakeConstants.intakeMotorName);
        // Direction: FORWARD (default)
        isRunning = true; // Intake runs continuously by default after initialization
    }

    /**
     * Starts the intake (sets isRunning to true).
     */
    public void startIntake() {
        isRunning = true;
    }

    /**
     * Stops the intake (sets isRunning to false).
     */
    public void stopIntake() {
        isRunning = false;
    }

    /**
     * Toggles the intake running state (On/Off).
     */
    public void toggle() {
        isRunning = !isRunning;
    }

    /**
     * Checks if the intake is currently running.
     * @return True if running.
     */
    public boolean isRunning() {
        return isRunning;
    }

    /**
     * Toggles the shooting state flag.
     * When shooting, intake might need to run at a specific power (transitPower).
     */
    public void setShooting(boolean shooting) {
        isShooting = shooting;
    }
    /**
     * Checks if the intake is in shooting mode.
     * @return True if shooting.
     */
    public boolean isShooting() {
        return isShooting;
    }

    /**
     * Sets the full power flag.
     * Used to override standard intake power with maximum power.
     *
     * @param fullPower True to enable full power.
     */

    public void setFullPower(boolean fullPower) {
        isFullPower = fullPower;
    }
    public void setFastIntaking(boolean fastIntakePower) {
        isFastIntaking = fastIntakePower;
    }
    /**
     * Sets the reversed flag.
     * Used to reverse the intake direction (e.g., for ejecting).
     *
     * @param reversed True to reverse intake.
     */
    public void setReversed(boolean reversed) {
        isReversed = reversed;
    }

    /**
     * Sets the fast shooting mode flag.
     * Used in auto when FAST shooting is active.
     *
     * @param fastShooting True to enable fast shooting power (0.8).
     */
    public void setFastShooting(boolean fastShooting) {
        isFastShooting = fastShooting;
    }
    
    /**
     * Sets the emergency disable flag.
     * When disabled, the intake motor will be set to 0 power.
     * Controlled by gamepad2 (LT + LB to toggle).
     *
     * @param disabled True to disable intake.
     */
    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
    }
    
    /**
     * Checks if the intake is disabled.
     * @return True if disabled.
     */
    public boolean isDisabled() {
        return disabled;
    }
    
    /**
     * Toggles the disabled state.
     */
    public void toggleDisabled() {
        disabled = !disabled;
    }

    /**
     * Gets the current velocity of the intake motor.
     * @return Velocity in ticks per second.
     */
    public double getVelocity() {
        return intakeMotor.getVelocity();
    }

    /**
     * Periodic update method.
     * Controls the motor power based on the current state flags.
     * Always runs at intakePower (0.65), unless reversed.
     */
    @Override
    public void periodic() {
        // Emergency disable check - highest priority
        if (disabled) {
            intakeMotor.setPower(0);
            return;
        }
        
        if (isRunning) {
            double targetPower;

            // Priority: fastShooting > fullPower > standard
            if (isShooting) {
                targetPower = IntakeConstants.transitPower; //1
            } else if (isFastIntaking){
                targetPower = IntakeConstants.fastIntakePower; // 0.7
            } else if (isFastShooting) {
                targetPower = IntakeConstants.fastShootingPower; // 0.8
            } else if (isFullPower) {
                targetPower = IntakeConstants.fullPower; // 0.65
            } else {
                targetPower = IntakeConstants.intakePower; // 0.5
            }

            // Reverse logic
            if (isReversed) {
                targetPower = -targetPower;
            }

            intakeMotor.setPower(targetPower);
        }
        else {
            intakeMotor.setPower(0);
        }
    }
}

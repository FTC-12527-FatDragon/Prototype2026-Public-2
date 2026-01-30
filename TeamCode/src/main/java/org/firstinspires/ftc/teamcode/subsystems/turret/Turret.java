package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem for the Turret (Gimbal) mechanism.
 * Controls a single motor for turret rotation.
 */
public class Turret extends SubsystemBase {
    public final DcMotor turretMotor;
    
    // Current power being applied
    private double targetPower = 0;

    /**
     * Constructor for Turret.
     * Initializes the turret motor.
     *
     * @param hardwareMap The hardware map from the OpMode.
     */
    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, TurretConstants.turretMotorName);
        
        // Configure motor
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Sets the turret motor power.
     * @param power Motor power (-1.0 to 1.0)
     */
    public void setPower(double power) {
        this.targetPower = Math.max(TurretConstants.minPower, 
                                    Math.min(TurretConstants.maxPower, power));
    }

    /**
     * Stops the turret motor.
     */
    public void stop() {
        this.targetPower = 0;
    }

    /**
     * Rotates the turret left (negative power).
     * @param speed Speed of rotation (0.0 to 1.0)
     */
    public void rotateLeft(double speed) {
        setPower(-Math.abs(speed));
    }

    /**
     * Rotates the turret right (positive power).
     * @param speed Speed of rotation (0.0 to 1.0)
     */
    public void rotateRight(double speed) {
        setPower(Math.abs(speed));
    }

    /**
     * Gets the current encoder position of the turret.
     * @return Encoder position in ticks.
     */
    public int getPosition() {
        return turretMotor.getCurrentPosition();
    }

    /**
     * Resets the encoder to zero.
     */
    public void resetEncoder() {
        DcMotor.RunMode currentMode = turretMotor.getMode();
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(currentMode);
    }

    /**
     * Gets the current target power.
     * @return Target power (-1.0 to 1.0)
     */
    public double getTargetPower() {
        return targetPower;
    }

    /**
     * Periodic update method.
     * Applies the target power to the motor.
     */
    @Override
    public void periodic() {
        turretMotor.setPower(targetPower);
    }
}



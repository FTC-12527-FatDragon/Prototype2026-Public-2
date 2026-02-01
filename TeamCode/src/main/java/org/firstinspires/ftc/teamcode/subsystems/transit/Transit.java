package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Subsystem handling the Transit (Feeder) mechanism.
 * Controls the servo that pushes rings/elements into the shooter flywheel.
 * Also controls a limit servo that opens when transit is up, closes when down.
 */
public class Transit extends SubsystemBase {
    public final Servo transitServo;
    public final Servo limitServo;

    // Default state is DOWN (retracted)
    public TransitState transitState = TransitState.DOWN;

    /**
     * Constructor for Transit.
     * Initializes the transit servo and limit servo.
     *
     * @param hardwareMap The hardware map.
     */
    public Transit(HardwareMap hardwareMap) {
        transitServo = hardwareMap.get(Servo.class, TransitConstants.transitServoName);
        limitServo = hardwareMap.get(Servo.class, TransitConstants.limitServoName);
    }

    /**
     * Enum representing the positions of the transit servo.
     */
    public enum TransitState {
        UP(TransitConstants.transitUpPos),   // Engaged/Pushing position
        DOWN(TransitConstants.transitDownPos); // Retracted/Resting position

        final double pos;

        TransitState(double transitPos) {
            pos = transitPos;
        }
    }

    /**
     * Sets the target state for the transit servo.
     * @param transitState The desired TransitState.
     */
    public void setTransitState(TransitState transitState) {
        this.transitState = transitState;
    }

    /**
     * Periodic update method.
     * Updates the servo positions to match the current state.
     * Limit servo automatically opens when transit is UP, closes when DOWN.
     */
    @Override
    public void periodic() {
        transitServo.setPosition(transitState.pos);
        
        // Limit servo follows transit state: open when UP, closed when DOWN
        if (transitState == TransitState.UP) {
            limitServo.setPosition(TransitConstants.limitOpenPos);
        } else {
            limitServo.setPosition(TransitConstants.limitClosedPos);
        }
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.

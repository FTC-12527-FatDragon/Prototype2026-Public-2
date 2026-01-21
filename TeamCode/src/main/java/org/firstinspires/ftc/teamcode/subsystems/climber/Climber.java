package org.firstinspires.ftc.teamcode.subsystems.climber;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Subsystem for the Climber mechanism.
 * Controls two servos (left and right) for climbing/hanging.
 */
public class Climber extends SubsystemBase {
    public final Servo leftClimberServo;
    public final Servo rightClimberServo;
    
    // Current state
    private ClimberState currentState = ClimberState.RETRACTED;

    /**
     * Constructor for Climber.
     * Initializes both climber servos.
     *
     * @param hardwareMap The hardware map from the OpMode.
     */
    public Climber(HardwareMap hardwareMap) {
        leftClimberServo = hardwareMap.get(Servo.class, ClimberConstants.leftClimberServoName);
        rightClimberServo = hardwareMap.get(Servo.class, ClimberConstants.rightClimberServoName);
    }

    /**
     * Enum representing the positions of the climber servos.
     */
    public enum ClimberState {
        RETRACTED,  // Both servos retracted (resting position)
        EXTENDED    // Both servos extended (climbing position)
    }

    /**
     * Sets the target state for both climber servos.
     * @param state The desired ClimberState.
     */
    public void setClimberState(ClimberState state) {
        this.currentState = state;
    }

    /**
     * Gets the current climber state.
     * @return The current ClimberState.
     */
    public ClimberState getClimberState() {
        return currentState;
    }

    /**
     * Extends both climber servos.
     */
    public void extend() {
        setClimberState(ClimberState.EXTENDED);
    }

    /**
     * Retracts both climber servos.
     */
    public void retract() {
        setClimberState(ClimberState.RETRACTED);
    }

    /**
     * Toggles the climber state between EXTENDED and RETRACTED.
     */
    public void toggle() {
        if (currentState == ClimberState.EXTENDED) {
            retract();
        } else {
            extend();
        }
    }

    /**
     * Sets the left servo to a specific position.
     * @param position Servo position (0.0 to 1.0)
     */
    public void setLeftPosition(double position) {
        leftClimberServo.setPosition(position);
    }

    /**
     * Sets the right servo to a specific position.
     * @param position Servo position (0.0 to 1.0)
     */
    public void setRightPosition(double position) {
        rightClimberServo.setPosition(position);
    }

    /**
     * Sets both servos to specific positions.
     * @param leftPos Left servo position (0.0 to 1.0)
     * @param rightPos Right servo position (0.0 to 1.0)
     */
    public void setPositions(double leftPos, double rightPos) {
        leftClimberServo.setPosition(leftPos);
        rightClimberServo.setPosition(rightPos);
    }

    /**
     * Periodic update method.
     * Updates servo positions to match the current state.
     */
    @Override
    public void periodic() {
        switch (currentState) {
            case EXTENDED:
                leftClimberServo.setPosition(ClimberConstants.leftExtendedPos);
                rightClimberServo.setPosition(ClimberConstants.rightExtendedPos);
                break;
            case RETRACTED:
            default:
                leftClimberServo.setPosition(ClimberConstants.leftRetractedPos);
                rightClimberServo.setPosition(ClimberConstants.rightRetractedPos);
                break;
        }
    }
}


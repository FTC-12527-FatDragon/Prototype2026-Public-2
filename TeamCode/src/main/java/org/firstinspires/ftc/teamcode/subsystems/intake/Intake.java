package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {
    public final DcMotor intakeMotor;

    public static boolean isRunning = false;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, IntakeConstants.intakeMotorName);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void toggle() {
        isRunning = !isRunning;
    }

    public boolean isRunning() {
        return isRunning;
    }

    @Override
    public void periodic() {
        if (isRunning) {
            intakeMotor.setPower(IntakeConstants.intakePower);
        }
        else {
            intakeMotor.setPower(0);
        }
    }
}

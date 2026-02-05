package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Simple test program to read turret encoder position.
 * Just displays the raw encoder value - no control, no movement.
 */
@TeleOp(name = "Turret Position Reader", group = "Test")
public class TurretPositionReader extends LinearOpMode {
    
    @Override
    public void runOpMode() {
        // Get turret motor directly
        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        
        // Setup telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        telemetry.addLine("=== TURRET POSITION READER ===");
        telemetry.addLine("Press START to begin reading");
        telemetry.addLine("(Encoder will reset to 0 on start)");
        telemetry.update();
        
        waitForStart();
        
        // Reset encoder to 0 on start (same as manual mode)
        turretMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        while (opModeIsActive()) {
            // Read raw encoder position
            int rawPosition = turretMotor.getCurrentPosition();
            
            // Calculate angle (8192 CPR, 116:22 gear ratio)
            double ticksPerDegree = 8192.0 * (116.0 / 22.0) / 360.0;
            double angleDeg = rawPosition / ticksPerDegree;
            
            // Display
            telemetry.addLine("=== TURRET ENCODER ===");
            telemetry.addData("Raw Position", rawPosition);
            telemetry.addData("Angle (deg)", String.format("%.2f", angleDeg));
            telemetry.addData("Ticks/Degree", String.format("%.2f", ticksPerDegree));
            telemetry.update();
        }
    }
}

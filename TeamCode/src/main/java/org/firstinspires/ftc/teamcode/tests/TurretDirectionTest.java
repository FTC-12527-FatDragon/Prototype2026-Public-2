package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

/**
 * Test OpMode to verify turret direction control.
 * 
 * Sequence:
 * 1. Wait for start
 * 2. Turn to +90° (right)
 * 3. Wait 2 seconds
 * 4. Turn to -90° (left)
 * 5. Wait 2 seconds
 * 6. Return to 0°
 */
@TeleOp(name = "Turret Direction Test", group = "Test")
public class TurretDirectionTest extends LinearOpMode {
    
    private Turret turret;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize turret
        turret = new Turret(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Calibrate (set current position as 0°)
        turret.resetEncoder();  // This also sets isCalibrated = true
        
        telemetry.addLine("=== TURRET DIRECTION TEST ===");
        telemetry.addLine("Make sure turret is facing FORWARD before starting!");
        telemetry.addLine("");
        telemetry.addLine("Sequence:");
        telemetry.addLine("1. Turn to +90° (RIGHT)");
        telemetry.addLine("2. Wait 2 seconds");
        telemetry.addLine("3. Turn to -90° (LEFT)");
        telemetry.addLine("4. Wait 2 seconds");
        telemetry.addLine("5. Return to 0° (CENTER)");
        telemetry.addLine("");
        telemetry.addLine("Press START to begin...");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            // ===== STEP 1: Turn to +90° (RIGHT) =====
            telemetry.addLine(">>> Turning to +90° (RIGHT)...");
            telemetry.update();
            
            turret.enableSoftLock(90);  // Target: +90°
            
            // Wait until at position or timeout
            long startTime = System.currentTimeMillis();
            while (opModeIsActive() && !isAtTarget(90) && (System.currentTimeMillis() - startTime < 3000)) {
                turret.periodic();
                updateTelemetry(90);
                sleep(20);
            }
            
            // Hold for 2 seconds
            telemetry.addLine(">>> At +90°, holding for 2 seconds...");
            telemetry.update();
            holdPosition(2000);
            
            // ===== STEP 2: Turn to -90° (LEFT) =====
            telemetry.addLine(">>> Turning to -90° (LEFT)...");
            telemetry.update();
            
            turret.enableSoftLock(-90);  // Target: -90°
            
            // Wait until at position or timeout
            startTime = System.currentTimeMillis();
            while (opModeIsActive() && !isAtTarget(-90) && (System.currentTimeMillis() - startTime < 5000)) {
                turret.periodic();
                updateTelemetry(-90);
                sleep(20);
            }
            
            // Hold for 2 seconds
            telemetry.addLine(">>> At -90°, holding for 2 seconds...");
            telemetry.update();
            holdPosition(2000);
            
            // ===== STEP 3: Return to 0° (CENTER) =====
            telemetry.addLine(">>> Returning to 0° (CENTER)...");
            telemetry.update();
            
            turret.enableSoftLock(0);  // Target: 0°
            
            // Wait until at position or timeout
            startTime = System.currentTimeMillis();
            while (opModeIsActive() && !isAtTarget(0) && (System.currentTimeMillis() - startTime < 3000)) {
                turret.periodic();
                updateTelemetry(0);
                sleep(20);
            }
            
            // ===== DONE =====
            telemetry.addLine("=== TEST COMPLETE ===");
            telemetry.addData("Final Angle", String.format("%.1f°", turret.getAngleDegrees()));
            telemetry.update();
            
            // Keep running to hold position
            while (opModeIsActive()) {
                turret.periodic();
                updateTelemetry(0);
                sleep(20);
            }
        }
        
        // Stop turret
        turret.disableLock();
        turret.setPower(0);
    }
    
    private boolean isAtTarget(double targetAngle) {
        double error = Math.abs(turret.getAngleDegrees() - targetAngle);
        return error <= TurretConstants.positionTolerance;
    }
    
    private void holdPosition(long durationMs) {
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startTime < durationMs)) {
            turret.periodic();
            sleep(20);
        }
    }
    
    private void updateTelemetry(double targetAngle) {
        double currentAngle = turret.getAngleDegrees();
        double error = targetAngle - currentAngle;
        
        telemetry.addLine("=== TURRET STATUS ===");
        telemetry.addData("Target", String.format("%.1f°", targetAngle));
        telemetry.addData("Current", String.format("%.1f°", currentAngle));
        telemetry.addData("Error", String.format("%.1f°", error));
        telemetry.addData("Encoder Ticks", turret.getEncoderPosition());
        telemetry.addData("Mode", turret.getLockMode());
        telemetry.addLine("");
        telemetry.addLine("Direction Check:");
        telemetry.addLine("+90° should be RIGHT of robot");
        telemetry.addLine("-90° should be LEFT of robot");
        telemetry.update();
    }
}

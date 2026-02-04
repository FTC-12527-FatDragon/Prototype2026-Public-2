package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Shooter Velocity Tuner - Dashboard-only control for tuning shooter velocity control
 * 
 * ALL CONTROL VIA DASHBOARD:
 * - Set targetVelocity to set target speed (TPS)
 * - Set enabled = true to start motors
 * - Adjust PIDF parameters: kP, kI, kD, kF
 * 
 * This is a pure velocity closed-loop tuner using PIDF control.
 */
@TeleOp(name = "Shooter PID Tuner", group = "test")
@Config
public class ShooterPIDTuner extends LinearOpMode {
    
    // ==================== TARGET ====================
    public static double targetVelocity = 0;    // Target velocity in TPS (positive)
    public static boolean enabled = false;       // Motor enable switch
    
    // ==================== PIDF PARAMETERS ====================
    public static double kP = 0.0005;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.00036;  // Feedforward: ~1/maxVelocityTPS ≈ 1/2800
    
    // ==================== TOLERANCE ====================
    public static double tolerance = 50;  // TPS - considered "on target" if within this
    
    // ==================== VELOCITY FILTER ====================
    public static double filterAlpha = 0.15;  // Lower = smoother but slower response (0.1-0.3)
    
    
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;
    private PIDFController pidfController;
    private FtcDashboard dashboard;
    
    // Fixed 50ms window velocity calculation (stable, always positive)
    private int windowStartPos = 0;
    private long windowStartTime = 0;
    private double velocity = 0;
    private static final long WINDOW_MS = 50;  // Calculate velocity every 50ms
    
    @Override
    public void runOpMode() {
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        
        pidfController = new PIDFController(kP, kI, kD, kF);
        dashboard = FtcDashboard.getInstance();
        
        telemetry.addLine("=== SHOOTER VELOCITY TUNER ===");
        telemetry.addLine("Dashboard-only control");
        telemetry.addLine("");
        telemetry.addLine("1. Set targetVelocity (TPS)");
        telemetry.addLine("2. Set enabled = true");
        telemetry.addLine("3. Adjust kP/kI/kD/kF");
        telemetry.update();
        
        waitForStart();
        
        // Initialize
        windowStartPos = rightShooter.getCurrentPosition();
        windowStartTime = System.currentTimeMillis();
        
        while (opModeIsActive()) {
            // Update PIDF from Dashboard
            pidfController.setPIDF(kP, kI, kD, kF);
            
            // Fixed 50ms window velocity calculation
            int currentPos = rightShooter.getCurrentPosition();
            long currentTime = System.currentTimeMillis();
            long elapsed = currentTime - windowStartTime;
            
            if (elapsed >= WINDOW_MS) {
                // Calculate velocity only every 50ms (stable)
                int deltaPos = Math.abs(currentPos - windowStartPos);  // Always positive
                double rawVelocity = deltaPos * 1000.0 / elapsed;      // TPS (ticks per second)
                
                // Light smoothing
                velocity = filterAlpha * rawVelocity + (1 - filterAlpha) * velocity;
                
                // Reset window
                windowStartPos = currentPos;
                windowStartTime = currentTime;
            }
            
            double currentVel = velocity;  // Always positive
            double sdkVelocity = Math.abs(rightShooter.getVelocity());  // For comparison
            double error = targetVelocity - currentVel;
            
            double power = 0;
            String state = "DISABLED";
            boolean onTarget = false;
            
            if (enabled && targetVelocity > 0) {
                // PIDF velocity control
                power = pidfController.calculate(currentVel, targetVelocity);
                power = Math.max(-1.0, Math.min(1.0, power));
                
                onTarget = Math.abs(error) <= tolerance;
                if (onTarget) {
                    state = "ON_TARGET";
                } else if (error > 0) {
                    state = "ACCEL";
                } else {
                    state = "DECEL";
                }
            } else {
                pidfController.reset();
            }
            
            // Apply power (left positive, right negative)
            leftShooter.setPower(power);
            rightShooter.setPower(-power);
            
            // Dashboard telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("ENABLED", enabled);
            packet.put("ON_TARGET", onTarget);
            packet.put("STATE", state);
            packet.put("targetVelocity", targetVelocity);
            packet.put("currentVelocity", currentVel);      // Manual calculated (always positive)
            packet.put("sdkVelocity", sdkVelocity);         // SDK |getVelocity()| for comparison
            packet.put("encoderPosition", currentPos);       // Raw encoder position
            packet.put("error", error);
            packet.put("power", power);
            packet.put("kP", kP);
            packet.put("kI", kI);
            packet.put("kD", kD);
            packet.put("kF", kF);
            packet.put("filterAlpha", filterAlpha);
            dashboard.sendTelemetryPacket(packet);
            
            // Driver Station telemetry
            telemetry.addData("ENABLED", enabled ? "YES ✅" : "NO ❌");
            telemetry.addData("ON TARGET", onTarget ? "YES ✅" : "NO");
            telemetry.addData("STATE", state);
            telemetry.addLine("---");
            telemetry.addData("Target", "%.0f TPS", targetVelocity);
            telemetry.addData("Current (calc)", "%.0f TPS", currentVel);
            telemetry.addData("SDK Velocity", "%.0f TPS", sdkVelocity);
            telemetry.addData("Encoder", "%d", currentPos);
            telemetry.addData("Error", "%.0f TPS", error);
            telemetry.addData("Tolerance", "%.0f TPS", tolerance);
            telemetry.addLine("---");
            telemetry.addData("Power", "%.4f", power);
            telemetry.addLine("---");
            telemetry.addLine("== PIDF ==");
            telemetry.addData("kP", "%.6f", kP);
            telemetry.addData("kI", "%.6f", kI);
            telemetry.addData("kD", "%.6f", kD);
            telemetry.addData("kF", "%.6f", kF);
            telemetry.update();
        }
        
        // Stop motors on exit
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.

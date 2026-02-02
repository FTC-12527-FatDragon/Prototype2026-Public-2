package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Turret Motor Tuner - Dashboard-only control for tuning turret PIDF
 * 
 * ALL CONTROL VIA DASHBOARD:
 * - Set targetPosition to move turret
 * - Adjust kP/kI/kD/kF to tune PIDF
 * - Set enabled = true to start motor
 * 
 * Gamepad A = Reset encoder to 0 (optional)
 */
@TeleOp(name = "Turret Motor Tuner", group = "test")
@Config
public class TurretMotorTuner extends LinearOpMode {
    
    // ==================== PIDF PARAMETERS (Tuned 2026-02-02) ====================
    public static double kP = 0.0004;
    public static double kI = 0.0;
    public static double kD = 0.0000185;
    public static double kF = 0.058;          // Static friction compensation (direction-aware)
    
    // ==================== CONTROL ====================
    public static double targetPosition = 0;  // Target in ticks
    public static boolean enabled = false;    // Motor enable switch
    public static boolean reverseMotor = true; // Tuned: motor direction REVERSED
    public static double tolerance = 100;     // Position tolerance (ticks) - ~0.83°
    
    // ==================== INFO ====================
    // REV Through Bore Encoder V2: 8192 CPR
    // Gear Ratio: 116:22 (motor:turret)
    public static double TICKS_PER_DEGREE = 8192.0 * (116.0 / 22.0) / 360.0;  // ≈ 120 ticks/degree
    
    private DcMotorEx turretMotor;
    private PIDFController pidfController;
    private FtcDashboard dashboard;
    private double startEncoderPos = 0;
    
    @Override
    public void runOpMode() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        pidfController = new PIDFController(kP, kI, kD, kF);
        dashboard = FtcDashboard.getInstance();
        
        telemetry.addLine("=== TURRET MOTOR TUNER ===");
        telemetry.addLine("Control via Dashboard only!");
        telemetry.addLine("Set 'enabled = true' to start");
        telemetry.addLine("[A] Reset encoder");
        telemetry.update();
        
        waitForStart();
        
        // Record startup position as zero reference
        startEncoderPos = turretMotor.getCurrentPosition();
        
        boolean lastReverseMotor = reverseMotor;
        
        while (opModeIsActive()) {
            // Update motor direction if changed in Dashboard
            if (reverseMotor != lastReverseMotor) {
                turretMotor.setDirection(reverseMotor ? 
                    DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
                lastReverseMotor = reverseMotor;
            }
            
            // Update PIDF from Dashboard
            pidfController.setPIDF(kP, kI, kD, kF);
            
            // Get positions
            double rawPos = turretMotor.getCurrentPosition();
            double deltaPos = rawPos - startEncoderPos;
            double velocity = turretMotor.getVelocity();
            double error = targetPosition - deltaPos;
            
            // A button: Reset encoder
            if (gamepad1.a) {
                startEncoderPos = rawPos;
                targetPosition = 0;
                pidfController.reset();
            }
            
            // Motor control
            double power = 0;
            boolean onTarget = Math.abs(error) <= tolerance;
            
            if (enabled) {
                if (onTarget) {
                    power = 0;  // Within tolerance, stop
                } else {
                    // Use PID only (kF = 0 in controller)
                    pidfController.setPIDF(kP, kI, kD, 0);  // Disable built-in F
                    double pidPower = pidfController.calculate(deltaPos, targetPosition);
                    
                    // Add F manually with correct direction (static friction compensation)
                    // F should push in the direction of error
                    double feedforward = (error > 0) ? kF : -kF;
                    
                    power = pidPower + feedforward;
                    power = Math.max(-1.0, Math.min(1.0, power));
                }
            }
            turretMotor.setPower(power);
            
            
            // Dashboard telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("ENABLED", enabled);
            packet.put("ON_TARGET", onTarget);
            packet.put("targetPosition", targetPosition);
            packet.put("currentPosition", deltaPos);
            packet.put("error", error);
            packet.put("tolerance", tolerance);
            packet.put("power", power);
            packet.put("velocity", velocity);
            packet.put("angle_deg", deltaPos / TICKS_PER_DEGREE);
            dashboard.sendTelemetryPacket(packet);
            
            // Driver Station telemetry
            telemetry.addData("ENABLED", enabled ? "YES ✅" : "NO ❌");
            telemetry.addData("ON TARGET", onTarget ? "YES ✅" : "NO");
            telemetry.addLine("---");
            telemetry.addData("Target", "%.0f ticks", targetPosition);
            telemetry.addData("Current", "%.0f ticks", deltaPos);
            telemetry.addData("Error", "%.0f ticks (tol: %.0f)", error, tolerance);
            telemetry.addData("Angle", "%.1f°", deltaPos / TICKS_PER_DEGREE);
            telemetry.addLine("---");
            telemetry.addData("Power", "%.4f", power);
            telemetry.addData("Velocity", "%.0f TPS", velocity);
            telemetry.addLine("---");
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);
            telemetry.update();
        }
        
        turretMotor.setPower(0);
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.

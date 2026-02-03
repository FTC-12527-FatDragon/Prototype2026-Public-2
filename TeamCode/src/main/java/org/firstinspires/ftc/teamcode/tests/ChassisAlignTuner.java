package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivePinpoint;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Chassis Heading PIDF Tuner
 * 
 * Robot tries to maintain starting heading (0°).
 * Manually turn the robot to test PIDF resistance.
 * 
 * Dashboard Controls:
 * - enabled: Start/stop PIDF
 * - kP, kI, kD, kF: PIDF coefficients
 */
@TeleOp(name = "Chassis Heading Tuner", group = "test")
@Config
public class ChassisAlignTuner extends LinearOpMode {
    
    // ==================== PID PARAMETERS (Tuned 2026-02-03) ====================
    public static double kP = 0.03;
    public static double kI = 0.0;
    public static double kD = 0.003;
    public static double kF = 0.0;
    
    // ==================== CONTROL ====================
    public static boolean enabled = true;  // Default ON
    public static double tolerance = 2.0;
    public static double maxPower = 1.0;
    public static double iMax = 50.0;
    public static double iZone = 30.0;
    
    // Hardware
    private MecanumDrivePinpoint drive;
    private FtcDashboard dashboard;
    
    // PIDF State
    private double headingOffset = 0;
    private double prevError = 0;
    private double totalError = 0;
    private long lastTimeNanos = 0;
    
    @Override
    public void runOpMode() {
        drive = new MecanumDrivePinpoint(hardwareMap);
        
        // Lock turret
        try {
            DcMotor turret = hardwareMap.get(DcMotor.class, "turretMotor");
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setPower(0);
        } catch (Exception ignored) {}
        
        dashboard = FtcDashboard.getInstance();
        
        telemetry.addLine("=== CHASSIS HEADING TUNER ===");
        telemetry.addLine("");
        telemetry.addLine("Robot will hold starting heading.");
        telemetry.addLine("Turn robot by hand to test PIDF.");
        telemetry.addLine("");
        telemetry.addLine("Tune kP/kI/kD/kF on Dashboard");
        telemetry.update();
        
        waitForStart();
        
        // Lock to current heading
        headingOffset = drive.getPose().getHeading(AngleUnit.DEGREES);
        
        while (opModeIsActive()) {
            // Current heading relative to start (target is always 0)
            double rawHeading = drive.getPose().getHeading(AngleUnit.DEGREES);
            double currentHeading = normalizeAngle(rawHeading - headingOffset);
            double error = currentHeading;  // Positive error = rotated right = need to turn left (negative power)
            
            // Calculate dt
            long now = System.nanoTime();
            double dt = (lastTimeNanos == 0) ? 0.02 : (now - lastTimeNanos) / 1.0E9;
            lastTimeNanos = now;
            
            // PIDF
            double turnPower = 0;
            boolean onTarget = Math.abs(error) <= tolerance;
            
            if (enabled && !onTarget) {
                double dError = (dt > 0) ? (error - prevError) / dt : 0;
                
                if (Math.abs(error) > iZone) {
                    totalError = 0;
                } else if (kI != 0) {
                    totalError += error * dt;
                    double limit = iMax / kI;
                    totalError = Math.max(-limit, Math.min(limit, totalError));
                }
                
                double pTerm = kP * error;
                double iTerm = kI * totalError;
                double dTerm = kD * dError;
                double base = pTerm + iTerm + dTerm;
                double fTerm = (Math.abs(base) > 1e-6) ? Math.signum(base) * kF : 0;
                
                turnPower = Math.max(-maxPower, Math.min(maxPower, base + fTerm));
            } else if (onTarget) {
                totalError = 0;
            }
            
            prevError = error;
            
            // Apply
            drive.moveRobotFieldRelative(0, 0, turnPower);
            
            // Dashboard
            TelemetryPacket p = new TelemetryPacket();
            p.put("enabled", enabled);
            p.put("onTarget", onTarget);
            p.put("currentHeading", currentHeading);
            p.put("error", error);
            p.put("turnPower", turnPower);
            dashboard.sendTelemetryPacket(p);
            
            // Telemetry
            telemetry.addData("ENABLED", enabled ? "YES ✅" : "NO ❌");
            telemetry.addData("ON TARGET", onTarget ? "YES ✅" : "NO");
            telemetry.addLine("---");
            telemetry.addData("Current", "%.1f°", currentHeading);
            telemetry.addData("Error", "%.1f°", error);
            telemetry.addData("Turn Power", "%.4f", turnPower);
            telemetry.addLine("---");
            telemetry.addData("kP", "%.5f", kP);
            telemetry.addData("kI", "%.5f", kI);
            telemetry.addData("kD", "%.5f", kD);
            telemetry.addData("kF", "%.3f", kF);
            telemetry.update();
        }
        
        drive.stop();
    }
    
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.

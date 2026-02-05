package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

/**
 * Turret PID Tuner - Test and tune turret PID parameters in real-time via FTC Dashboard.
 * 
 * Controls:
 * - A: Go to +120°
 * - B: Go to -120°
 * - X: Go to +180°
 * - Y: Go to 0°
 * - D-pad Up: Go to target angle (Dashboard adjustable)
 * - D-pad Down: Reset encoder to 0
 * 
 * Dashboard Tuning:
 * - TurretPIDTuner.targetAngle: Target angle in degrees
 * - TurretConstants.kP_deg, kI_deg, kD_deg, kF_deg: PID parameters
 */
@Config
@TeleOp(name = "Turret PID Tuner", group = "Tests")
public class TurretPIDTuner extends LinearOpMode {
    
    // Dashboard tunable target angle
    public static double targetAngle = 0.0;
    
    private DcMotorEx turretMotor;
    private PIDFController pidController;
    
    // Encoder constants
    private static final double ENCODER_CPR = 8192.0;
    private static final double GEAR_RATIO = 116.0 / 22.0;
    private static final double TICKS_PER_DEGREE = (ENCODER_CPR * GEAR_RATIO) / 360.0;
    
    private double desiredAngle = 0.0;
    private boolean pidEnabled = false;
    
    @Override
    public void runOpMode() {
        // Initialize telemetry with Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize motor
        turretMotor = hardwareMap.get(DcMotorEx.class, TurretConstants.turretMotorName);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        if (TurretConstants.reverseMotor) {
            turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        
        // Initialize PID controller
        pidController = new PIDFController(
                TurretConstants.kP_deg,
                TurretConstants.kI_deg,
                TurretConstants.kD_deg,
                0
        );
        
        telemetry.addLine("=== Turret PID Tuner ===");
        telemetry.addLine("A: +120°  B: -120°  X: +180°  Y: 0°");
        telemetry.addLine("D-pad Up: Go to targetAngle");
        telemetry.addLine("D-pad Down: Reset encoder");
        telemetry.addLine("");
        telemetry.addLine("Tune in Dashboard: TurretConstants");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Update PID parameters from Dashboard (real-time tuning!)
            pidController.setPIDF(
                    TurretConstants.kP_deg,
                    TurretConstants.kI_deg,
                    TurretConstants.kD_deg,
                    0
            );
            
            // Get current angle
            double currentTicks = turretMotor.getCurrentPosition();
            double currentAngle = currentTicks / TICKS_PER_DEGREE;
            
            // Button controls (larger range for testing)
            if (gamepad1.a) {
                desiredAngle = 120.0;
                pidEnabled = true;
            } else if (gamepad1.b) {
                desiredAngle = -120.0;
                pidEnabled = true;
            } else if (gamepad1.x) {
                desiredAngle = 180.0;
                pidEnabled = true;
            } else if (gamepad1.y) {
                desiredAngle = 0.0;
                pidEnabled = true;
            } else if (gamepad1.dpad_up) {
                desiredAngle = targetAngle;  // From Dashboard
                pidEnabled = true;
            } else if (gamepad1.dpad_down) {
                // Reset encoder
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pidEnabled = false;
                desiredAngle = 0.0;
            }
            
            // PID control
            double power = 0;
            double error = desiredAngle - currentAngle;
            
            if (pidEnabled) {
                // Calculate PID output
                double pidOutput = pidController.calculate(currentAngle, desiredAngle);
                
                // Add feedforward for static friction
                double feedforward = 0;
                if (Math.abs(error) > 1.0) {  // Only add FF outside deadband
                    feedforward = (error > 0) ? TurretConstants.kF_deg : -TurretConstants.kF_deg;
                }
                
                power = pidOutput + feedforward;
                power = Math.max(-1.0, Math.min(1.0, power));
                
                // Stop if close enough
                if (Math.abs(error) < 0.5) {
                    power = 0;
                }
            }
            
            turretMotor.setPower(power);
            
            // Telemetry
            telemetry.addData("=== PID Parameters ===", "");
            telemetry.addData("kP_deg", "%.6f", TurretConstants.kP_deg);
            telemetry.addData("kI_deg", "%.6f", TurretConstants.kI_deg);
            telemetry.addData("kD_deg", "%.6f", TurretConstants.kD_deg);
            telemetry.addData("kF_deg", "%.3f", TurretConstants.kF_deg);
            telemetry.addLine("");
            telemetry.addData("=== Status ===", "");
            telemetry.addData("Target Angle", "%.2f°", desiredAngle);
            telemetry.addData("Current Angle", "%.2f°", currentAngle);
            telemetry.addData("Error", "%.2f°", error);
            telemetry.addData("Power", "%.3f", power);
            telemetry.addData("PID Enabled", pidEnabled);
            telemetry.addLine("");
            telemetry.addData("Encoder Ticks", "%.0f", currentTicks);
            telemetry.addData("Dashboard targetAngle", "%.1f°", targetAngle);
            telemetry.update();
        }
        
        turretMotor.setPower(0);
    }
}

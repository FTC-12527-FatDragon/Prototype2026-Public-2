package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Chassis Align Tuner - For tuning heading PIDF (rotation control)
 * 
 * Uses GoBilda Pinpoint for heading measurement.
 * F term is applied manually based on error direction for static friction compensation.
 * 
 * Dashboard Controls:
 * - targetHeading: Target angle in degrees (0 = forward)
 * - enabled: Enable PIDF control
 * - kP, kI, kD, kF: PIDF coefficients
 * 
 * Gamepad:
 * - A: Reset heading to 0
 * - D-Pad Left/Right: Adjust target ±15°
 */
@TeleOp(name = "Chassis Align Tuner", group = "test")
@Config
public class ChassisAlignTuner extends LinearOpMode {
    
    // ==================== PIDF PARAMETERS ====================
    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.005;
    public static double kF = 0.05;  // Static friction compensation (applied based on error direction)
    
    // ==================== CONTROL ====================
    public static double targetHeading = 0;   // Target heading in degrees
    public static boolean enabled = false;    // Enable rotation control
    public static double tolerance = 2.0;     // Degrees - stop when within this range
    public static double maxPower = 0.5;      // Max rotation power
    
    // Hardware
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor turretMotor;  // Keep turret locked during test
    private com.qualcomm.hardware.gobilda.GoBildaPinpointDriver pinpoint;
    private PIDFController pidfController;
    private FtcDashboard dashboard;
    
    private double headingOffset = 0;  // For resetting heading
    
    @Override
    public void runOpMode() {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBack = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBack = hardwareMap.get(DcMotor.class, "rightBackMotor");
        
        // Set motor directions (typical mecanum setup)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize turret motor - keep it locked during chassis tuning
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setPower(0);  // Brake mode, no power
        
        // Initialize Pinpoint
        pinpoint = hardwareMap.get(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.class, "od");
        pinpoint.setOffsets(DriveConstants.xPoseDW, DriveConstants.yPoseDW, 
                           org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
        pinpoint.setEncoderResolution(
            com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
            com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.REVERSED,
            com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
        
        // Initialize PIDF (F is applied manually based on error direction)
        pidfController = new PIDFController(kP, kI, kD, 0);
        dashboard = FtcDashboard.getInstance();
        
        telemetry.addLine("=== CHASSIS ALIGN TUNER ===");
        telemetry.addLine("Control via Dashboard!");
        telemetry.addLine("Set 'enabled = true' to start");
        telemetry.addLine("[A] Reset heading");
        telemetry.addLine("[D-Pad L/R] ±15°");
        telemetry.update();
        
        waitForStart();
        
        // Record initial heading
        pinpoint.update();
        headingOffset = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
        
        boolean lastDpadLeft = false;
        boolean lastDpadRight = false;
        
        while (opModeIsActive()) {
            // Update Pinpoint
            pinpoint.update();
            
            // Update PIDF from Dashboard (F is applied manually)
            pidfController.setPIDF(kP, kI, kD, 0);
            
            // Get current heading (relative to start)
            double rawHeading = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
            double currentHeading = normalizeAngle(rawHeading - headingOffset);
            
            // Calculate error (shortest path)
            double error = normalizeAngle(targetHeading - currentHeading);
            
            // A button: Reset heading
            if (gamepad1.a) {
                headingOffset = rawHeading;
                targetHeading = 0;
                pidfController.reset();
            }
            
            // D-Pad: Adjust target
            if (gamepad1.dpad_left && !lastDpadLeft) {
                targetHeading = normalizeAngle(targetHeading - 15);
            }
            lastDpadLeft = gamepad1.dpad_left;
            
            if (gamepad1.dpad_right && !lastDpadRight) {
                targetHeading = normalizeAngle(targetHeading + 15);
            }
            lastDpadRight = gamepad1.dpad_right;
            
            // PIDF Control
            double turnPower = 0;
            boolean onTarget = Math.abs(error) <= tolerance;
            
            if (enabled && !onTarget) {
                // PID calculation - use (current, target) like turret does
                double pidPower = pidfController.calculate(currentHeading, targetHeading);
                
                // Add F manually with direction awareness (static friction compensation)
                double feedforward = (error > 0) ? kF : -kF;
                turnPower = pidPower + feedforward;
                
                // Clamp to max power
                turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));
            }
            
            // Apply to motors (turn only, no translation)
            leftFront.setPower(turnPower);
            leftBack.setPower(turnPower);
            rightFront.setPower(-turnPower);
            rightBack.setPower(-turnPower);
            
            // Dashboard telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("ENABLED", enabled);
            packet.put("ON_TARGET", onTarget);
            packet.put("targetHeading", targetHeading);
            packet.put("currentHeading", currentHeading);
            packet.put("error", error);
            packet.put("turnPower", turnPower);
            packet.put("tolerance", tolerance);
            dashboard.sendTelemetryPacket(packet);
            
            // Driver Station telemetry
            telemetry.addData("ENABLED", enabled ? "YES ✅" : "NO ❌");
            telemetry.addData("ON TARGET", onTarget ? "YES ✅" : "NO");
            telemetry.addLine("---");
            telemetry.addData("Target", "%.1f°", targetHeading);
            telemetry.addData("Current", "%.1f°", currentHeading);
            telemetry.addData("Error", "%.1f° (tol: %.1f°)", error, tolerance);
            telemetry.addLine("---");
            telemetry.addData("Turn Power", "%.4f", turnPower);
            telemetry.addLine("---");
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);
            telemetry.update();
        }
        
        // Stop motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    
    /**
     * Normalize angle to [-180, 180]
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.

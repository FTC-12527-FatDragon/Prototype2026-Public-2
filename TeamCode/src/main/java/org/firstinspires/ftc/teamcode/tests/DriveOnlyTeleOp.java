package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Drive-Only TeleOp for testing drivetrain.
 * Only uses 4 drive motors + Pinpoint odometry.
 * Field-centric Mecanum drive with same logic as main TeleOp.
 * 
 * Controls:
 * - Left Stick: Move (field-centric)
 * - Right Stick X: Rotate
 * - Left Stick Click: Reset heading
 */
@Config
@TeleOp(name = "Drive Only Test", group = "Test")
public class DriveOnlyTeleOp extends LinearOpMode {
    
    // Drive motors
    private DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    
    // Pinpoint odometry
    private GoBildaPinpointDriver pinpoint;
    
    // Heading offset for field-centric reset
    private double yawOffset = 0;
    
    // Tunable constants (same as DriveConstants)
    public static double strafingBalance = 1.1;
    public static double deadband = 0.03;
    
    // Pinpoint offsets (in inches)
    public static double xPoseDW = -93.45 / 25.4;
    public static double yPoseDW = 24.05 / 25.4;
    
    @Override
    public void runOpMode() {
        // Initialize motors
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        
        // Initialize Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "od");
        
        // Motor brake mode
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Motor directions (same as MecanumDrivePinpoint)
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // Pinpoint configuration
        pinpoint.setOffsets(xPoseDW, yPoseDW, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();
        
        telemetry.addLine("Drive Only Test Ready");
        telemetry.addLine("Left Stick: Move | Right Stick: Turn");
        telemetry.addLine("Left Stick Click: Reset Heading");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Update Pinpoint
            pinpoint.update();
            
            // Reset heading on left stick button
            if (gamepad1.left_stick_button) {
                yawOffset = pinpoint.getHeading(AngleUnit.RADIANS);
            }
            
            // Get raw inputs (same as TeleOpDriveCommand)
            double rawLeftX = -gamepad1.left_stick_x;
            double rawLeftY = -gamepad1.left_stick_y;  // Negate Y (up = positive)
            double rawRightX = gamepad1.right_stick_x;
            
            // Apply squared input curve
            double forward = rawLeftY * Math.abs(rawLeftY);
            double strafe = rawLeftX * Math.abs(rawLeftX);  // No negation here
            double turn = rawRightX * Math.abs(rawRightX);
            
            // Apply deadband
            if (Math.abs(forward) < deadband) forward = 0;
            if (Math.abs(strafe) < deadband) strafe = 0;
            
            // Check if any input
            boolean hasInput = forward != 0 || strafe != 0 || Math.abs(turn) > deadband;
            
            if (hasInput) {
                // Field-centric transformation (same as MecanumDrivePinpoint)
                double botHeading = pinpoint.getHeading(AngleUnit.RADIANS) - yawOffset;
                
                double rotX = strafe * Math.cos(botHeading) - forward * Math.sin(botHeading);
                double rotY = strafe * Math.sin(botHeading) + forward * Math.cos(botHeading);
                
                // Apply strafing balance with negation (same as MecanumDrivePinpoint)
                rotX = -rotX * strafingBalance;
                
                // Calculate motor powers (Mecanum kinematics)
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
                double leftFrontPower = (rotY + rotX + turn) / denominator;
                double leftBackPower = (rotY - rotX + turn) / denominator;
                double rightFrontPower = (rotY - rotX - turn) / denominator;
                double rightBackPower = (rotY + rotX - turn) / denominator;
                
                // Set motor powers
                leftFrontMotor.setPower(leftFrontPower);
                leftBackMotor.setPower(leftBackPower);
                rightFrontMotor.setPower(rightFrontPower);
                rightBackMotor.setPower(rightBackPower);
            } else {
                // Stop all motors
                leftFrontMotor.setPower(0);
                leftBackMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightBackMotor.setPower(0);
            }
            
            // Telemetry
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(pinpoint.getHeading(AngleUnit.RADIANS) - yawOffset));
            telemetry.addData("Pos X", "%.2f in", pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("Pos Y", "%.2f in", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addLine();
            telemetry.addData("LF Power", "%.2f", leftFrontMotor.getPower());
            telemetry.addData("LB Power", "%.2f", leftBackMotor.getPower());
            telemetry.addData("RF Power", "%.2f", rightFrontMotor.getPower());
            telemetry.addData("RB Power", "%.2f", rightBackMotor.getPower());
            telemetry.update();
        }
    }
}


package org.firstinspires.ftc.teamcode.opmodes.teleops;

/**
 * EMERGENCY BACKUP - Minimal Version
 * No sensors, pure open-loop, robot-centric
 * 
 * Drivetrain: Robot-centric (headed) mode
 * Shooter: Pure open-loop power, no velocity feedback
 * Turret: D-Pad left/right manual control
 * Intake: LT to activate
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "!!! EMERGENCY !!!", group = "!Emergency")
public class SoloEmergency extends LinearOpMode {
    
    // Drivetrain
    DcMotor fl, fr, bl, br;
    
    // Shooter
    DcMotor shooterL, shooterR;
    Servo shooterServo;
    
    // Intake
    DcMotor intake;
    
    // Transit
    Servo transit, limit;
    
    // Turret
    DcMotor turret;
    
    // ========== Open-loop Power Settings ==========
    // Shooter power (0-1)
    double SLOW_POWER = 0.3;
    double MID_POWER = 0.45;
    double FAST_POWER = 0.6;
    double IDLE_POWER = 0.27;
    
    // Shooter angle
    double SLOW_ANGLE = 0.04;
    double MID_ANGLE = 0.5;
    double FAST_ANGLE = 1.0;
    
    // Transit
    double TRANSIT_UP = 0.36;
    double TRANSIT_DOWN = 0.62;
    double LIMIT_OPEN = 0.6;
    double LIMIT_CLOSE = 0.3;
    
    // Turret speed
    double TURRET_SPEED = 0.3;
    
    // Shooter spin-up timing
    double SPINUP_TIME = 1.0;  // Spin-up wait time (seconds)
    double shooterStartTime = 0;
    boolean wasShooterOn = false;
    
    @Override
    public void runOpMode() {
        
        // ========== Initialization ==========
        // Drivetrain
        fl = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        fr = hardwareMap.get(DcMotor.class, "frontRightMotor");
        bl = hardwareMap.get(DcMotor.class, "backLeftMotor");
        br = hardwareMap.get(DcMotor.class, "backRightMotor");
        
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Shooter
        shooterL = hardwareMap.get(DcMotor.class, "leftShooterMotor");
        shooterR = hardwareMap.get(DcMotor.class, "rightShooterMotor");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        
        // Intake
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        
        // Transit
        transit = hardwareMap.get(Servo.class, "transitServo");
        limit = hardwareMap.get(Servo.class, "limitServo");
        
        // Turret
        turret = hardwareMap.get(DcMotor.class, "turretMotor");
        
        telemetry.addLine("=== EMERGENCY MODE ===");
        telemetry.addLine("NO SENSORS - OPEN LOOP");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            // ========== Drivetrain - Robot-centric (headed) ==========
            double y = -gamepad1.left_stick_y;  // Forward/back
            double x = gamepad1.left_stick_x;   // Left/right
            double r = gamepad1.right_stick_x;  // Rotation
            
            // Mecanum formula
            fl.setPower(y + x + r);
            fr.setPower(y - x - r);
            bl.setPower(y - x + r);
            br.setPower(y + x - r);
            
            // ========== Shooter - Pure Open-loop ==========
            double shootPower = IDLE_POWER;
            double servoAngle = MID_ANGLE;
            boolean shooterOn = false;
            
            if (gamepad1.left_bumper) {
                // Near shot
                shootPower = SLOW_POWER;
                servoAngle = SLOW_ANGLE;
                shooterOn = true;
            } else if (gamepad1.right_bumper) {
                // Mid shot
                shootPower = MID_POWER;
                servoAngle = MID_ANGLE;
                shooterOn = true;
            } else if (gamepad1.right_trigger > 0.3) {
                // Far shot
                shootPower = FAST_POWER;
                servoAngle = FAST_ANGLE;
                shooterOn = true;
            }
            
            // Record when Shooter starts spinning up
            if (shooterOn && !wasShooterOn) {
                // Just started spinning up
                shooterStartTime = getRuntime();
            }
            if (!shooterOn) {
                // Shooter stopped, reset timer
                shooterStartTime = 0;
            }
            wasShooterOn = shooterOn;
            
            // Calculate elapsed spin-up time
            double spinupElapsed = shooterOn ? (getRuntime() - shooterStartTime) : 0;
            boolean shooterReady = spinupElapsed >= SPINUP_TIME;
            
            shooterL.setPower(shootPower);
            shooterR.setPower(-shootPower);
            shooterServo.setPosition(servoAngle);
            
            // ========== Intake ==========
            if (gamepad1.left_trigger > 0.3) {
                intake.setPower(1.0);
            } else if (gamepad1.dpad_up) {
                intake.setPower(-1.0);  // Reverse
            } else {
                intake.setPower(0.5);   // Default low speed
            }
            
            // ========== Transit ==========
            // Condition: LT + shoot button + Shooter spun up for 1 sec
            boolean wantToShoot = gamepad1.left_trigger > 0.3 && 
                (gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.right_trigger > 0.3);
            boolean canShoot = wantToShoot && shooterReady;
            
            if (canShoot) {
                transit.setPosition(TRANSIT_UP);
                limit.setPosition(LIMIT_OPEN);
            } else {
                transit.setPosition(TRANSIT_DOWN);
                limit.setPosition(LIMIT_CLOSE);
            }
            
            // ========== Turret - D-Pad Manual ==========
            if (gamepad1.dpad_left) {
                turret.setPower(-TURRET_SPEED);
            } else if (gamepad1.dpad_right) {
                turret.setPower(TURRET_SPEED);
            } else {
                turret.setPower(0);
            }
            
            // ========== Telemetry ==========
            telemetry.addLine("=== !!! EMERGENCY !!! ===");
            telemetry.addLine("");
            telemetry.addData("Shooter", shooterOn ? "ON" : "IDLE");
            telemetry.addData("Power", String.format("%.2f", shootPower));
            
            // Display spin-up status
            if (shooterOn) {
                if (shooterReady) {
                    telemetry.addData("Status", "✓ READY TO FIRE");
                } else {
                    telemetry.addData("Status", String.format("Spinning up... %.1fs", spinupElapsed));
                }
            } else {
                telemetry.addData("Status", "IDLE");
            }
            
            telemetry.addData("Transit", canShoot ? "FIRING!" : (wantToShoot ? "WAIT..." : "READY"));
            telemetry.addLine("");
            telemetry.addLine("LB=Near RB=Mid RT=Far");
            telemetry.addLine("LT=Intake+Fire(wait 1s)");
            telemetry.addLine("D-Pad L/R=Turret");
            telemetry.update();
        }
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.

package org.firstinspires.ftc.teamcode.opmodes.teleops;

/**
 * EMERGENCY BACKUP - 最简化版本
 * 无传感器，纯开环，机器人坐标系
 * 
 * 底盘：有头模式（机器人坐标系）
 * Shooter：纯开环功率，无速度检测
 * 云台：D-Pad 左右手动控制
 * Intake：LT 开启
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "!!! EMERGENCY !!!", group = "!Emergency")
public class SoloEmergency extends LinearOpMode {
    
    // 底盘
    DcMotor fl, fr, bl, br;
    
    // Shooter
    DcMotor shooterL, shooterR;
    Servo shooterServo;
    
    // Intake
    DcMotor intake;
    
    // Transit
    Servo transit, limit;
    
    // 云台
    DcMotor turret;
    
    // ========== 开环功率设置 ==========
    // Shooter 功率（0-1）
    double SLOW_POWER = 0.3;
    double MID_POWER = 0.45;
    double FAST_POWER = 0.6;
    double IDLE_POWER = 0.27;
    
    // Shooter 角度
    double SLOW_ANGLE = 0.04;
    double MID_ANGLE = 0.5;
    double FAST_ANGLE = 1.0;
    
    // Transit
    double TRANSIT_UP = 0.36;
    double TRANSIT_DOWN = 0.62;
    double LIMIT_OPEN = 0.6;
    double LIMIT_CLOSE = 0.3;
    
    // 云台速度
    double TURRET_SPEED = 0.3;
    
    // Shooter 加速计时
    double SPINUP_TIME = 1.0;  // 加速等待时间（秒）
    double shooterStartTime = 0;
    boolean wasShooterOn = false;
    
    @Override
    public void runOpMode() {
        
        // ========== 初始化 ==========
        // 底盘
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
        
        // 云台
        turret = hardwareMap.get(DcMotor.class, "turretMotor");
        
        telemetry.addLine("=== EMERGENCY MODE ===");
        telemetry.addLine("NO SENSORS - OPEN LOOP");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            // ========== 底盘 - 机器人坐标系（有头） ==========
            double y = -gamepad1.left_stick_y;  // 前后
            double x = gamepad1.left_stick_x;   // 左右
            double r = gamepad1.right_stick_x;  // 旋转
            
            // 麦轮公式
            fl.setPower(y + x + r);
            fr.setPower(y - x - r);
            bl.setPower(y - x + r);
            br.setPower(y + x - r);
            
            // ========== Shooter - 纯开环 ==========
            double shootPower = IDLE_POWER;
            double servoAngle = MID_ANGLE;
            boolean shooterOn = false;
            
            if (gamepad1.left_bumper) {
                // 近射
                shootPower = SLOW_POWER;
                servoAngle = SLOW_ANGLE;
                shooterOn = true;
            } else if (gamepad1.right_bumper) {
                // 中射
                shootPower = MID_POWER;
                servoAngle = MID_ANGLE;
                shooterOn = true;
            } else if (gamepad1.right_trigger > 0.3) {
                // 远射
                shootPower = FAST_POWER;
                servoAngle = FAST_ANGLE;
                shooterOn = true;
            }
            
            // 记录 Shooter 开始加速的时间
            if (shooterOn && !wasShooterOn) {
                // 刚开始加速
                shooterStartTime = getRuntime();
            }
            if (!shooterOn) {
                // Shooter 停了，重置计时
                shooterStartTime = 0;
            }
            wasShooterOn = shooterOn;
            
            // 计算已加速时间
            double spinupElapsed = shooterOn ? (getRuntime() - shooterStartTime) : 0;
            boolean shooterReady = spinupElapsed >= SPINUP_TIME;
            
            shooterL.setPower(shootPower);
            shooterR.setPower(-shootPower);
            shooterServo.setPosition(servoAngle);
            
            // ========== Intake ==========
            if (gamepad1.left_trigger > 0.3) {
                intake.setPower(1.0);
            } else if (gamepad1.dpad_up) {
                intake.setPower(-1.0);  // 反转
            } else {
                intake.setPower(0.5);   // 默认低速
            }
            
            // ========== Transit ==========
            // 条件：LT + 射击键 + Shooter 已加速 1 秒
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
            
            // ========== 云台 - D-Pad 手动 ==========
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
            
            // 显示加速状态
            if (shooterOn) {
                if (shooterReady) {
                    telemetry.addData("Status", "✓ READY TO FIRE");
                } else {
                    telemetry.addData("Status", String.format("加速中... %.1fs", spinupElapsed));
                }
            } else {
                telemetry.addData("Status", "IDLE");
            }
            
            telemetry.addData("Transit", canShoot ? "FIRING!" : (wantToShoot ? "WAIT..." : "READY"));
            telemetry.addLine("");
            telemetry.addLine("LB=近 RB=中 RT=远");
            telemetry.addLine("LT=进球+发射(需等1秒)");
            telemetry.addLine("D-Pad L/R=云台");
            telemetry.update();
        }
    }
}

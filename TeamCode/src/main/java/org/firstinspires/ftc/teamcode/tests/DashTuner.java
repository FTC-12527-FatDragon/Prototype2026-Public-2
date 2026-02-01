package org.firstinspires.ftc.teamcode.tests;

/**
 * Dashboard Tuner OpMode.
 * For tuning shooter PID, servo positions, and other parameters via FTC Dashboard.
 */

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;
import java.util.Queue;

class PIDF {
    public double kP, kI, kD, kF;

    PIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
}

@TeleOp(name = "DashTuner")
@Config
public class DashTuner extends LinearOpMode {
    public static String[] motorName = {"", "", "", ""};
    public static String[] servoName = {"", "", "", ""};
    public static boolean[] closeLoop = new boolean[4];
    public static double[] motorTarget = new double[4];
    public static double[] servoTarget = new double[4];
    public static double[] slaveTo = {-1, -1, -1, -1};

    public static boolean[] isVelocityCloseLoop = new boolean[4];

    public static PIDF[] PIDFs = {
            new PIDF(0, 0, 0, 0),
            new PIDF(0, 0, 0, 0),
            new PIDF(0, 0, 0, 0),
            new PIDF(0, 0, 0, 0)
    };

    DcMotorEx[] motors = new DcMotorEx[4];

    Servo[] servos = new Servo[4];

    PIDFController[] pidfControllers = {
            new PIDFController(0, 0, 0, 0),
            new PIDFController(0, 0, 0, 0),
            new PIDFController(0, 0, 0, 0),
            new PIDFController(0, 0, 0, 0)
    };
    
    // ==================== ENCODER SAFETY PROTECTION ====================
    // If encoder doesn't change for 0.5s while motor is running, STOP!
    public static double ENCODER_TIMEOUT = 0.5;  // seconds
    public static double ENCODER_MIN_CHANGE = 5;  // minimum encoder change to be considered "moving"
    public static boolean resetEncoderError = false;  // Set to true in Dashboard to reset encoder errors
    
    private double[] lastEncoderPos = new double[4];
    private double[] encoderStuckStartTime = new double[4];
    private boolean[] encoderStuckDetected = new boolean[4];

    public static String colorSensorName = "";

    ColorSensor colorSensor;

    DistanceSensor distanceSensor;

    float hsvValues[] = {0F, 0F, 0F};

    final float values[] = hsvValues;

    final double SCALE_FACTOR = 255;

    private boolean ballDetected = false;
    private boolean purpule = false;
    private boolean green = false;

    private double lastR = 0.0, lastG = 0.0, lastB = 0.0;

    private Queue<Integer> colorQue = new LinkedList<>();

    List<Float> hues = new ArrayList<>();

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        for (int i = 0; i < 4; i++) {
            if (!motorName[i].isEmpty()) {
                motors[i] = hardwareMap.get(DcMotorEx.class, motorName[i]);
                pidfControllers[i].setPIDF(PIDFs[i].kP, PIDFs[i].kI, PIDFs[i].kD, PIDFs[i].kF);
            }
            if (!servoName[i].isEmpty()) {
                servos[i] = hardwareMap.get(Servo.class, servoName[i]);
            }
        }

        if (!colorSensorName.isEmpty()) {
            colorSensor = hardwareMap.get(ColorSensor.class, colorSensorName);
            distanceSensor = hardwareMap.get(DistanceSensor.class, colorSensorName);
        }

        waitForStart();
        
        // Initialize encoder positions to prevent false positives on first loop
        for (int i = 0; i < 4; i++) {
            if (motors[i] != null) {
                lastEncoderPos[i] = motors[i].getCurrentPosition();
            }
            encoderStuckStartTime[i] = 0;
            encoderStuckDetected[i] = false;
        }

        while (opModeIsActive()) {
            // Check for encoder error reset request from Dashboard
            if (resetEncoderError) {
                for (int i = 0; i < 4; i++) {
                    encoderStuckDetected[i] = false;
                    encoderStuckStartTime[i] = 0;
                    if (motors[i] != null) {
                        lastEncoderPos[i] = motors[i].getCurrentPosition();
                    }
                }
                resetEncoderError = false;
                telemetry.addLine("Encoder errors cleared!");
                telemetry.update();
            }
            
            for (int i = 0; i < 4; i++) {
                if (!motorName[i].isEmpty()) {
                    if (slaveTo[i] != -1) {
                        if (closeLoop[i])
                            motors[i].setPower(-pidfControllers[(int) slaveTo[i]].calculate(motors[(int)
                                    slaveTo[i]].getVelocity(), motorTarget[(int) slaveTo[i]]));
                        else
                            motors[i].setPower(-motorTarget[(int) slaveTo[i]]);
                    }
                    else if (closeLoop[i] && isVelocityCloseLoop[i]) {
                        // Velocity closed-loop with PIDF
                        pidfControllers[i].setPIDF(PIDFs[i].kP, PIDFs[i].kI, PIDFs[i].kD, PIDFs[i].kF);

                        double v = motors[i].getVelocity();

                        motors[i].setPower(pidfControllers[i].calculate(v, motorTarget[i]));

                        TelemetryPacket packet = new TelemetryPacket();
                        packet.put("targetVelocity " + i, motorTarget[i]);
                        packet.put("Velocity " + i, v);
                        packet.put("TPS " + i, v);  // Ticks Per Second

                        dashboard.sendTelemetryPacket(packet);
                    }
                    else if (closeLoop[i] && !isVelocityCloseLoop[i]) {
                        // Position closed-loop with PIDF
                        pidfControllers[i].setPIDF(PIDFs[i].kP, PIDFs[i].kI, PIDFs[i].kD, PIDFs[i].kF);

                        double pos = motors[i].getCurrentPosition();
                        double v = motors[i].getVelocity();  // Get velocity for TPS
                        double currentTime = getRuntime();
                        
                        // ===== ENCODER SAFETY CHECK =====
                        // If encoder hasn't changed for ENCODER_TIMEOUT seconds, STOP!
                        double encoderChange = Math.abs(pos - lastEncoderPos[i]);
                        double power = pidfControllers[i].calculate(pos, motorTarget[i]);
                        
                        if (Math.abs(power) > 0.05) {  // Motor is trying to move
                            if (encoderChange < ENCODER_MIN_CHANGE) {
                                // Encoder not changing
                                if (encoderStuckStartTime[i] == 0) {
                                    encoderStuckStartTime[i] = currentTime;
                                } else if (currentTime - encoderStuckStartTime[i] > ENCODER_TIMEOUT) {
                                    // ENCODER STUCK! EMERGENCY STOP!
                                    encoderStuckDetected[i] = true;
                                    motors[i].setPower(0);
                                    
                                    TelemetryPacket errorPacket = new TelemetryPacket();
                                    errorPacket.put("!!! ENCODER ERROR " + i + " !!!", "STOPPED - No encoder change for " + ENCODER_TIMEOUT + "s");
                                    errorPacket.put("Motor " + i + " DISABLED", true);
                                    dashboard.sendTelemetryPacket(errorPacket);
                                    
                                    telemetry.addLine("!!! ENCODER " + i + " STUCK !!!");
                                    telemetry.addLine("Motor stopped for safety!");
                                    telemetry.addData("Last Position", lastEncoderPos[i]);
                                    telemetry.addData("Current Position", pos);
                                    telemetry.update();
                                    continue;  // Skip this motor
                                }
                            } else {
                                // Encoder is changing, reset timer
                                encoderStuckStartTime[i] = 0;
                            }
                        } else {
                            // Motor not trying to move, reset timer
                            encoderStuckStartTime[i] = 0;
                        }
                        
                        lastEncoderPos[i] = pos;
                        
                        // If encoder stuck was detected, keep motor stopped
                        if (encoderStuckDetected[i]) {
                            motors[i].setPower(0);
                            TelemetryPacket errorPacket = new TelemetryPacket();
                            errorPacket.put("Motor " + i + " DISABLED", "Encoder error - restart OpMode to reset");
                            dashboard.sendTelemetryPacket(errorPacket);
                            continue;
                        }
                        
                        motors[i].setPower(power);

                        TelemetryPacket packet = new TelemetryPacket();
                        packet.put("targetPosition " + i, motorTarget[i]);
                        packet.put("currentPosition " + i, pos);
                        packet.put("TPS " + i, v);  // Ticks Per Second
                        packet.put("encoderOK " + i, !encoderStuckDetected[i]);

                        dashboard.sendTelemetryPacket(packet);
                    }
                    else if (!closeLoop[i]) {
                        motors[i].setPower(motorTarget[i]);
                        double v = motors[i].getVelocity();

                        TelemetryPacket packet = new TelemetryPacket();
                        packet.put("Power " + i, motorTarget[i]);
                        packet.put("TPS " + i, v);  // Ticks Per Second

                        dashboard.sendTelemetryPacket(packet);
                    }
                }

                if (!servoName[i].isEmpty()) {
                    servos[i].setPosition(servoTarget[i]);
                }

                if (!colorSensorName.isEmpty()) {
                    Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                            (int) (colorSensor.green() * SCALE_FACTOR),
                            (int) (colorSensor.blue() * SCALE_FACTOR),
                            hsvValues);

                    TelemetryPacket packet = new TelemetryPacket();

                    packet.put("Alpha", colorSensor.alpha());
                    packet.put("Red  ", colorSensor.red());
                    packet.put("Green", colorSensor.green());
                    packet.put("Blue ", colorSensor.blue());
                    packet.put("Hue", hsvValues[0]);
                    packet.put("Distance (cm)", String.format(Locale.US, "%.02f", distanceSensor.getDistance(DistanceUnit.CM)));

                    double dis = distanceSensor.getDistance(DistanceUnit.CM);

                    double r = colorSensor.red();
                    double g = colorSensor.green();
                    double b = colorSensor.blue();

                    Color.RGBToHSV((int) (r * SCALE_FACTOR),
                            (int) (g * SCALE_FACTOR),
                            (int) (b * SCALE_FACTOR),
                            hsvValues);

                    if (dis < 4.0) {
                        hues.add(hsvValues[0]);
                        ballDetected = true;
                    }

                    if (dis > 4.0 && ballDetected) {
                        ballDetected = false;

                        Collections.sort(hues);

                        float res = hues.get(hues.size() / 2);

                        if (res >= 180) {
                            colorQue.offer(1);
                        }
                        else {
                            colorQue.offer(0);
                        }

                        purpule = false;
                        green = false;

                        hues.clear();
                    }

                    packet.put("BALL COLORS", colorQue);

                    dashboard.sendTelemetryPacket(packet);

                    lastG = g; lastB = b; lastR = r;
                }
            }
        }
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.

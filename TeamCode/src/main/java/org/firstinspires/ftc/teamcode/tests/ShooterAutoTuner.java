package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.List;

/**
 * Shooter PIDF Auto-Tuner (Fully Automatic)
 * 
 * Press START and the program automatically:
 * 1. Feedforward Test: Find steady-state power → kF
 * 2. Step Response Test: Measure rise time, overshoot → initial kP/kD
 * 3. Fine-tune: Adjust based on response characteristics
 * 4. Display final PIDF parameters
 * 
 * Set targetVelocity on Dashboard before starting.
 */
@TeleOp(name = "Shooter Auto Tuner", group = "test")
@Config
public class ShooterAutoTuner extends LinearOpMode {
    
    // ==================== TEST PARAMETERS ====================
    public static double targetVelocity = 400000;  // Target velocity (TPS)
    
    // ==================== CALCULATED RESULTS ====================
    public static double calculated_kP = 0;
    public static double calculated_kI = 0;
    public static double calculated_kD = 0;
    public static double calculated_kF = 0;
    
    // ==================== TEST RESULTS ====================
    public static double riseTime = 0;
    public static double overshoot = 0;
    public static double settlingTime = 0;
    public static double steadyStateError = 0;
    public static double steadyStatePower = 0;
    public static double maxVelocityReached = 0;
    
    // Hardware
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;
    private FtcDashboard dashboard;
    
    // Velocity calculation (50ms window)
    private int windowStartPos = 0;
    private long windowStartTime = 0;
    private double velocity = 0;
    private static final long WINDOW_MS = 50;
    private static final double FILTER_ALPHA = 0.15;
    
    // Test phases
    private enum Phase {
        WAITING,           // Waiting for user to start
        COOLDOWN_1,        // Let motor stop before test
        FEEDFORWARD,       // Find kF
        COOLDOWN_2,        // Let motor stop
        STEP_RESPONSE,     // Measure response
        COOLDOWN_3,        // Let motor stop
        VERIFICATION,      // Test with calculated parameters
        COMPLETE           // Done!
    }
    private Phase phase = Phase.WAITING;
    private long phaseStartTime = 0;
    
    // Data recording
    private List<Double> velocityHistory = new ArrayList<>();
    private List<Long> timeHistory = new ArrayList<>();
    
    // Feedforward search state
    private double ffPower = 0;
    private long ffStableStart = 0;
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        dashboard = FtcDashboard.getInstance();
        
        telemetry.addLine("=== SHOOTER AUTO TUNER ===");
        telemetry.addLine("");
        telemetry.addLine("Set targetVelocity on Dashboard");
        telemetry.addLine("Then press START");
        telemetry.addLine("");
        telemetry.addLine("Program will automatically:");
        telemetry.addLine("1. Find feedforward (kF)");
        telemetry.addLine("2. Measure step response");
        telemetry.addLine("3. Calculate kP/kI/kD");
        telemetry.addLine("4. Verify with test run");
        telemetry.update();
        
        waitForStart();
        
        // Initialize
        windowStartPos = rightShooter.getCurrentPosition();
        windowStartTime = System.currentTimeMillis();
        phase = Phase.COOLDOWN_1;
        phaseStartTime = System.currentTimeMillis();
        
        while (opModeIsActive()) {
            // Update velocity
            updateVelocity();
            
            // Run current phase
            long phaseElapsed = System.currentTimeMillis() - phaseStartTime;
            
            switch (phase) {
                case COOLDOWN_1:
                case COOLDOWN_2:
                case COOLDOWN_3:
                    runCooldown(phaseElapsed);
                    break;
                case FEEDFORWARD:
                    runFeedforwardTest(phaseElapsed);
                    break;
                case STEP_RESPONSE:
                    runStepResponseTest(phaseElapsed);
                    break;
                case VERIFICATION:
                    runVerificationTest(phaseElapsed);
                    break;
                case COMPLETE:
                    // Motors off, display results
                    leftShooter.setPower(0);
                    rightShooter.setPower(0);
                    break;
                default:
                    break;
            }
            
            // Update telemetry
            updateTelemetry();
        }
        
        // Stop motors
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }
    
    private void updateVelocity() {
        int currentPos = rightShooter.getCurrentPosition();
        long currentTime = System.currentTimeMillis();
        long elapsed = currentTime - windowStartTime;
        
        if (elapsed >= WINDOW_MS) {
            int deltaPos = Math.abs(currentPos - windowStartPos);
            double rawVelocity = deltaPos * 1000.0 / elapsed;
            velocity = FILTER_ALPHA * rawVelocity + (1 - FILTER_ALPHA) * velocity;
            windowStartPos = currentPos;
            windowStartTime = currentTime;
        }
    }
    
    private void nextPhase(Phase next) {
        phase = next;
        phaseStartTime = System.currentTimeMillis();
        velocityHistory.clear();
        timeHistory.clear();
    }
    
    // ==================== COOLDOWN ====================
    private void runCooldown(long elapsed) {
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        
        // Wait 3 seconds for motor to fully stop
        if (elapsed > 3000) {
            velocity = 0;  // Reset velocity reading
            
            if (phase == Phase.COOLDOWN_1) {
                nextPhase(Phase.FEEDFORWARD);
                ffPower = 0.3;
                ffStableStart = 0;
            } else if (phase == Phase.COOLDOWN_2) {
                nextPhase(Phase.STEP_RESPONSE);
            } else if (phase == Phase.COOLDOWN_3) {
                nextPhase(Phase.VERIFICATION);
            }
        }
    }
    
    // ==================== FEEDFORWARD TEST ====================
    // Iterative search to find power that maintains target velocity
    // Takes time to converge - up to 60 seconds
    private void runFeedforwardTest(long elapsed) {
        // Apply current test power
        leftShooter.setPower(ffPower);
        rightShooter.setPower(-ffPower);
        
        // Wait for velocity to stabilize (at least 2 seconds)
        if (elapsed < 2000) return;
        
        double error = targetVelocity - velocity;
        double errorPercent = Math.abs(error) / targetVelocity;
        
        // Adjust power based on error
        if (errorPercent > 0.10) {  // More than 10% error: aggressive adjustment
            double adjustment = error * 0.000001;
            ffPower += adjustment;
            ffPower = Math.max(0.1, Math.min(1.0, ffPower));
            ffStableStart = 0;
        } else if (errorPercent > 0.05) {  // 5-10% error: medium adjustment
            double adjustment = error * 0.0000003;
            ffPower += adjustment;
            ffPower = Math.max(0.1, Math.min(1.0, ffPower));
            ffStableStart = 0;
        } else if (errorPercent > 0.02) {  // 2-5% error: fine adjustment
            double adjustment = error * 0.0000001;
            ffPower += adjustment;
            ffPower = Math.max(0.1, Math.min(1.0, ffPower));
            if (ffStableStart == 0) {
                ffStableStart = System.currentTimeMillis();
            }
        } else {
            // Within 2% tolerance
            if (ffStableStart == 0) {
                ffStableStart = System.currentTimeMillis();
            }
            
            // Stable for 5 seconds = done (longer to ensure true steady state)
            if (System.currentTimeMillis() - ffStableStart > 5000) {
                steadyStatePower = ffPower;
                calculated_kF = ffPower / targetVelocity;
                nextPhase(Phase.COOLDOWN_2);
            }
        }
        
        // Timeout after 60 seconds (1 minute)
        if (elapsed > 60000) {
            steadyStatePower = ffPower;
            calculated_kF = ffPower / targetVelocity;
            nextPhase(Phase.COOLDOWN_2);
        }
    }
    
    // ==================== STEP RESPONSE TEST ====================
    private void runStepResponseTest(long elapsed) {
        // Apply full power step
        leftShooter.setPower(1.0);
        rightShooter.setPower(-1.0);
        
        // Record data
        velocityHistory.add(velocity);
        timeHistory.add(elapsed);
        
        // Track max velocity
        if (velocity > maxVelocityReached) {
            maxVelocityReached = velocity;
        }
        
        // Run for 30 seconds (plenty of time for steady state)
        if (elapsed > 30000) {
            analyzeStepResponse();
            calculatePID();
            nextPhase(Phase.COOLDOWN_3);
        }
    }
    
    private void analyzeStepResponse() {
        if (velocityHistory.isEmpty()) return;
        
        double target90 = targetVelocity * 0.9;
        double target10 = targetVelocity * 0.1;
        
        // Calculate average final velocity (last 5 seconds for stability)
        double finalVelocity = 0;
        int finalCount = 0;
        for (int i = velocityHistory.size() - 1; i >= 0 && timeHistory.get(i) > 25000; i--) {
            finalVelocity += velocityHistory.get(i);
            finalCount++;
        }
        if (finalCount > 0) {
            finalVelocity /= finalCount;
        } else {
            finalVelocity = velocityHistory.get(velocityHistory.size() - 1);
        }
        
        // Find rise time (10% to 90%)
        long time10 = 0, time90 = 0;
        for (int i = 0; i < velocityHistory.size(); i++) {
            double v = velocityHistory.get(i);
            if (v >= target10 && time10 == 0) {
                time10 = timeHistory.get(i);
            }
            if (v >= target90 && time90 == 0) {
                time90 = timeHistory.get(i);
            }
        }
        riseTime = (time90 - time10) / 1000.0;
        if (riseTime < 0) riseTime = 0.1;  // Minimum 0.1s
        
        // Calculate overshoot relative to steady-state velocity (not target)
        // This is more accurate for systems that don't reach target
        if (finalVelocity > 0) {
            overshoot = Math.max(0, (maxVelocityReached - finalVelocity) / finalVelocity * 100);
        }
        
        // Steady state error (compared to target)
        steadyStateError = (targetVelocity - finalVelocity) / targetVelocity * 100;
        
        // Find settling time (within 5% of final steady-state velocity)
        double lowerBound = finalVelocity * 0.95;
        double upperBound = finalVelocity * 1.05;
        settlingTime = 30.0;  // Default to max test time
        for (int i = velocityHistory.size() - 1; i >= 0; i--) {
            double v = velocityHistory.get(i);
            if (v < lowerBound || v > upperBound) {
                if (i < velocityHistory.size() - 1) {
                    settlingTime = timeHistory.get(i + 1) / 1000.0;
                }
                break;
            }
        }
    }
    
    private void calculatePID() {
        // Based on step response characteristics and feedforward
        
        // kF already calculated from feedforward test
        // Now calculate kP, kI, kD
        
        if (riseTime > 0 && maxVelocityReached > 0) {
            // kP: Based on desired response speed
            // Faster rise time = can use smaller kP
            // Target: 0.3-0.5 second rise time
            double targetRiseTime = 0.4;
            double riseTimeFactor = targetRiseTime / Math.max(riseTime, 0.1);
            
            // Base kP from system gain
            double systemGain = maxVelocityReached / 1.0;  // velocity per unit power
            calculated_kP = riseTimeFactor / systemGain;
            
            // Scale down for safety
            calculated_kP *= 0.3;
            
            // kD: Reduce overshoot
            // More overshoot = more kD needed
            if (overshoot > 20) {
                calculated_kD = calculated_kP * riseTime * 0.15;
            } else if (overshoot > 10) {
                calculated_kD = calculated_kP * riseTime * 0.10;
            } else if (overshoot > 5) {
                calculated_kD = calculated_kP * riseTime * 0.05;
            } else {
                calculated_kD = calculated_kP * riseTime * 0.02;
            }
            
            // kI: Only if steady-state error is significant
            if (Math.abs(steadyStateError) > 3) {
                // Small kI to eliminate steady-state error
                calculated_kI = calculated_kP * 0.005;
            } else {
                calculated_kI = 0;
            }
        }
        
        // Sanity check - ensure reasonable values
        calculated_kP = Math.max(0.00001, Math.min(0.001, calculated_kP));
        calculated_kI = Math.max(0, Math.min(0.0001, calculated_kI));
        calculated_kD = Math.max(0, Math.min(0.0001, calculated_kD));
        calculated_kF = Math.max(0.0000001, Math.min(0.00001, calculated_kF));
    }
    
    // ==================== VERIFICATION TEST ====================
    // Test with calculated parameters
    private double verifyIntegral = 0;
    private double verifyLastError = 0;
    private long verifyLastTime = 0;
    
    private void runVerificationTest(long elapsed) {
        // Simple PIDF control with calculated parameters
        double error = targetVelocity - velocity;
        
        // Calculate dt
        long now = System.currentTimeMillis();
        double dt = (verifyLastTime == 0) ? 0.02 : (now - verifyLastTime) / 1000.0;
        verifyLastTime = now;
        
        // PID terms
        double pTerm = calculated_kP * error;
        
        verifyIntegral += error * dt;
        verifyIntegral = Math.max(-100000, Math.min(100000, verifyIntegral));
        double iTerm = calculated_kI * verifyIntegral;
        
        double dTerm = 0;
        if (dt > 0) {
            dTerm = calculated_kD * (error - verifyLastError) / dt;
        }
        verifyLastError = error;
        
        double fTerm = calculated_kF * targetVelocity;
        
        double power = pTerm + iTerm + dTerm + fTerm;
        power = Math.max(0, Math.min(1, power));
        
        leftShooter.setPower(power);
        rightShooter.setPower(-power);
        
        // Record for analysis
        velocityHistory.add(velocity);
        timeHistory.add(elapsed);
        
        // Run for 30 seconds (plenty of time to verify)
        if (elapsed > 30000) {
            // Analyze verification results
            double avgError = 0;
            int count = 0;
            // Look at last 10 seconds (steady state) - timeHistory > 20000 means last 10s of 30s test
            for (int i = velocityHistory.size() - 1; i >= 0 && timeHistory.get(i) > 20000; i--) {
                avgError += Math.abs(targetVelocity - velocityHistory.get(i));
                count++;
            }
            if (count > 0) {
                avgError /= count;
                double avgErrorPercent = avgError / targetVelocity * 100;
                
                // Adjust parameters based on verification results
                if (avgErrorPercent > 10) {
                    // Error too high, increase kP and kF
                    calculated_kP *= 1.5;
                    calculated_kF *= 1.1;
                } else if (avgErrorPercent > 5) {
                    calculated_kP *= 1.2;
                }
                
                // Update steady state error with verification results
                // Average last 5 seconds of data
                double finalAvgVel = 0;
                int finalCount = 0;
                for (int i = velocityHistory.size() - 1; i >= 0 && timeHistory.get(i) > 25000; i--) {
                    finalAvgVel += velocityHistory.get(i);
                    finalCount++;
                }
                if (finalCount > 0) {
                    finalAvgVel /= finalCount;
                    steadyStateError = (targetVelocity - finalAvgVel) / targetVelocity * 100;
                }
            }
            
            nextPhase(Phase.COMPLETE);
        }
    }
    
    private void updateTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("phase", phase.toString());
        packet.put("velocity", velocity);
        packet.put("targetVelocity", targetVelocity);
        packet.put("calc_kP", calculated_kP);
        packet.put("calc_kI", calculated_kI);
        packet.put("calc_kD", calculated_kD);
        packet.put("calc_kF", calculated_kF);
        dashboard.sendTelemetryPacket(packet);
        
        telemetry.addLine("=== SHOOTER AUTO TUNER ===");
        telemetry.addData("Phase", phase);
        telemetry.addData("Velocity", "%.0f TPS", velocity);
        telemetry.addData("Target", "%.0f TPS", targetVelocity);
        telemetry.addLine("");
        
        if (phase == Phase.COMPLETE) {
            telemetry.addLine("========== RESULTS ==========");
            telemetry.addLine("Copy these to ShooterConstants:");
            telemetry.addLine("");
            telemetry.addData("kP", "%.8f", calculated_kP);
            telemetry.addData("kI", "%.8f", calculated_kI);
            telemetry.addData("kD", "%.8f", calculated_kD);
            telemetry.addData("kF", "%.8f", calculated_kF);
            telemetry.addLine("");
            telemetry.addLine("--- Test Results ---");
            telemetry.addData("Rise Time", "%.3f s", riseTime);
            telemetry.addData("Overshoot", "%.1f %%", overshoot);
            telemetry.addData("Settling", "%.3f s", settlingTime);
            telemetry.addData("SS Error", "%.1f %%", steadyStateError);
            telemetry.addData("SS Power", "%.3f", steadyStatePower);
        } else {
            telemetry.addLine("--- Progress ---");
            switch (phase) {
                case COOLDOWN_1:
                case COOLDOWN_2:
                case COOLDOWN_3:
                    telemetry.addLine("Cooling down...");
                    break;
                case FEEDFORWARD:
                    telemetry.addLine("Finding feedforward (kF)...");
                    telemetry.addData("Test Power", "%.3f", ffPower);
                    break;
                case STEP_RESPONSE:
                    telemetry.addLine("Measuring step response...");
                    telemetry.addData("Max Velocity", "%.0f", maxVelocityReached);
                    break;
                case VERIFICATION:
                    telemetry.addLine("Verifying parameters...");
                    break;
                default:
                    break;
            }
        }
        
        telemetry.update();
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.

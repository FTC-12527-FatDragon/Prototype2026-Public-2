package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing Configuration.
 * Defines FollowerConstants, DriveConstants, and LocalizerConstants (Pinpoint).
 */
@Config
public class Constants {

    // Follower Constants: Define mass, limits, and PIDF coefficients for path following
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.1) // Robot mass in kg
            .forwardZeroPowerAcceleration(DriveConstants.forwardAcceleration)
            .lateralZeroPowerAcceleration(DriveConstants.strafeAcceleration)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.14, 0.000, 0.007, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.02, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.008, 0, 0.0001, 0, 0.02))
            .centripetalScaling(0.0003);


    // Mecanum Drivetrain Constants: Motor names, directions, and velocity limits
    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName(DriveConstants.leftFrontMotorName)
            .leftRearMotorName(DriveConstants.leftBackMotorName)
            .rightFrontMotorName(DriveConstants.rightFrontMotorName)
            .rightRearMotorName(DriveConstants.rightBackMotorName)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(DriveConstants.forwardVelocity)
            .yVelocity(DriveConstants.strafeVelocity);

    // Pinpoint Localizer Constants
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(DriveConstants.yPoseDW)
            .strafePodX(DriveConstants.xPoseDW)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("od")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    
    /**
     * Constraints for Path Generation (commented out/unused in current builder, but good for reference)
     * Order: tValue, velocity, translational, heading, timeout, brakingStrength, searchLimit, brakingStart
     */
//    public static PathConstraints pathConstraints = new PathConstraints(
//            0.995,
//            0.1,
//            0.1,
//            0.009,
//            50,
//            1.25,
//            10,
//            1
//    );

    /**
     * Factory method to create a Follower instance.
     * Builds the follower with Mecanum Drive and Pinpoint Localizer.
     *
     * @param hardwareMap The hardware map.
     * @return Configured Follower instance.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
//                .pathConstraints(pathConstraints)
                .build();
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.

package org.firstinspires.ftc.teamcode.pedropathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.frozenmilk.sinister.loading.Pinned;

@Configurable
@Pinned
public class PedroConstants {

    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(14.52)
        .forwardZeroPowerAcceleration(-52.59001607765112)
        .lateralZeroPowerAcceleration(-75.95069102915481)
        .useSecondaryTranslationalPIDF(true)
        .useSecondaryHeadingPIDF(true)
        .useSecondaryDrivePIDF(true)
        .translationalPIDFCoefficients(new PIDFCoefficients(0.11, 0, 0.011, 0.0))
        .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.15,0,0.01,0))
        .headingPIDFCoefficients(new PIDFCoefficients(2.5, 0, 0.15, 0.0))
        .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.5, 0, 0.15, 0.0))
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.001, 0.8, 0))
        .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.0005, 0.6, 0))
        .centripetalScaling(0.0003);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
        .maxPower(1)
        .rightFrontMotorName("rightFront")
        .rightRearMotorName("rightBack")
        .leftRearMotorName("leftBack")
        .leftFrontMotorName("leftFront")
        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .xVelocity(71.86471293291709)
        .yVelocity(64.43372399037278)
        .useBrakeModeInTeleOp(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
        .forwardPodY(4.181)
        .strafePodX(-2.113)
        .distanceUnit(DistanceUnit.INCH)
        .hardwareMapName("odo")
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
            .pathConstraints(pathConstraints)
            .mecanumDrivetrain(driveConstants)
            .pinpointLocalizer(localizerConstants)
            .build();
    }
}
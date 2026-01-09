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
public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(13)
        .forwardZeroPowerAcceleration(-34.07191846121948)
        .lateralZeroPowerAcceleration(-69.48189711179306)
        .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.005, 0.0))
        .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.2, 0.0))
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0, 0.8, 0))
        .centripetalScaling(0.002);

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
        .xVelocity(60.65308914785311)
        .yVelocity(58.99270750030759)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
        .forwardPodY(-2)
        .strafePodX(-4)
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
package org.firstinspires.ftc.teamcode.pedro;

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
        .mass(5)
        .forwardZeroPowerAcceleration(-59.11991561998617)
        .lateralZeroPowerAcceleration(-74.34097668640246)
        .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.007, 0.02))
        .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.035, 0.02))
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.001, 0.6, 0.02))
        .centripetalScaling(0.002);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
        .maxPower(1)
        .rightFrontMotorName("rightFront")
        .rightRearMotorName("rightBack")
        .leftRearMotorName("leftBack")
        .leftFrontMotorName("leftFront")
        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        .xVelocity(62.860017070620074)
        .yVelocity(54.91674948865034);

    public static PinpointConstants localizerConstants = new PinpointConstants()
        .forwardPodY(-6.6141732283)
        .strafePodX(2.3622047244)
        .distanceUnit(DistanceUnit.INCH)
        .hardwareMapName("odo")
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
            .pathConstraints(pathConstraints)
            .mecanumDrivetrain(driveConstants)
            .pinpointLocalizer(localizerConstants)
            .build();
    }
}
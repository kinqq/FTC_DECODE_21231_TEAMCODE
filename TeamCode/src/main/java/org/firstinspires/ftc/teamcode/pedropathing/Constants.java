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
        .mass(12.2)
        .forwardZeroPowerAcceleration(-52.31008506587686)
        .lateralZeroPowerAcceleration(-86.6952747252473)
        .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.02, 0.03))
        .headingPIDFCoefficients(new PIDFCoefficients(2.5, 0, 0.2, 0.03))
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.005, 0.8, 0.03))
        .centripetalScaling(0.0008);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
        .maxPower(1)
        .rightFrontMotorName("rightFront")
        .rightRearMotorName("rightBack")
        .leftRearMotorName("leftBack")
        .leftFrontMotorName("leftFront")
        .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .xVelocity(72.64601339505414)
        .yVelocity(52.427703136534205)
        .useBrakeModeInTeleOp(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
        .forwardPodY(-1.8897637795)
        .strafePodX(-7.25)
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
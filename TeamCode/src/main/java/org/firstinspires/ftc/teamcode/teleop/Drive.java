package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.Magazine;
import static org.firstinspires.ftc.teamcode.subsystem.Magazine.*;

import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.DetectedColor;
import org.firstinspires.ftc.teamcode.util.Slot;

@TeleOp(name = "Drive")
@Configurable
public class Drive extends OpMode {
    private GoBildaPinpointDriver odo;
    private DcMotorEx frontLeft, backLeft, frontRight, backRight, intake, launcher;
    private Magazine mag;
    private Turret turret;
    private AllianceColor allianceColor = AllianceColor.RED;
    public static double power = 0;

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        //TODO: Fine tune
        odo.setOffsets(-48, -182.5, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        odo.resetPosAndIMU();

        // Declare motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        mag = new Magazine(hardwareMap);
        turret = new Turret(hardwareMap);

        PanelsConfigurables.INSTANCE.refreshClass(Drive.class);

        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        telemetry.addLine("Initialized!");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) allianceColor = AllianceColor.BLUE;
        if (gamepad1.bWasPressed()) allianceColor = AllianceColor.RED;

        telemetry.addData("Alliance", allianceColor);
        telemetry.update();
    }

    @Override
    public void start() {
        turret.zeroHere();
        turret.setLaunchAngle(25);
    }

    @Override
    public void loop() {
        odo.update();
        turret.update();


        if (gamepad1.startWasPressed()) odo.resetPosAndIMU();
        double heading = -odo.getHeading(AngleUnit.RADIANS);
//        heading = 0; // for testing

        double y = -gamepad1.left_stick_y; // Y stick is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double xSpeed = x * Math.cos(heading) - y * Math.sin(heading);
        double ySpeed = x * Math.sin(heading) + y * Math.cos(heading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (ySpeed + xSpeed + rx) / denominator;
        double backLeftPower = (ySpeed - xSpeed + rx) / denominator;
        double frontRightPower = (ySpeed - xSpeed - rx) / denominator;
        double backRightPower = (ySpeed + xSpeed - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        double xDistanceFromGoal = Constants.RED_GOAL_X - odo.getPosX(DistanceUnit.MM) + 150;
        double yDistanceFromGoal = Constants.RED_GOAL_Y + odo.getPosY(DistanceUnit.MM);
        double angleFromPosition = Math.atan2(xDistanceFromGoal, yDistanceFromGoal) / Math.PI * 180;
        double angleFromHeading = -odo.getHeading(AngleUnit.DEGREES);

        if (allianceColor == AllianceColor.RED) turret.setTarget(angleFromHeading - angleFromPosition);
        if (allianceColor == AllianceColor.BLUE) turret.setTarget(angleFromHeading + angleFromPosition);

        if (gamepad2.dpadLeftWasPressed()) turret.adjustTarget(-1);
        if (gamepad2.dpadRightWasPressed()) turret.adjustTarget(1);
        if (gamepad2.dpadUpWasPressed()) turret.setLaunchAngle(turret.getLaunchAngle() + 10);
        if (gamepad2.dpadDownWasPressed()) turret.setLaunchAngle(turret.getLaunchAngle() - 10);

        if (gamepad1.dpadDownWasPressed()) power = Constants.INTAKE_ACTIVE_POWER;
        if (gamepad1.dpadUpWasPressed()) power = -Constants.INTAKE_ACTIVE_POWER;
        if (gamepad1.dpadLeftWasPressed()) power = 0;
        intake.setPower(power);



        if (gamepad2.xWasPressed()) launcher.setPower(launcher.getPower() == 1 ? 0 : 1);
        if (gamepad2.aWasPressed()) mag.startLaunch(DetectedColor.GREEN);
        if (gamepad2.bWasPressed()) mag.startLaunch(DetectedColor.PURPLE);
        if (gamepad2.yWasPressed()) mag.startLaunch(mag.getSlot());

        mag.update();

        telemetry.addData("x", odo.getPosX(DistanceUnit.MM));
        telemetry.addData("y", odo.getPosY(DistanceUnit.MM));
        telemetry.addData("h", odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("slot1", mag.getLastColor(Slot.FIRST));
        telemetry.addData("slot2", mag.getLastColor(Slot.SECOND));
        telemetry.addData("slot3", mag.getLastColor(Slot.THIRD));
        telemetry.addData("velocity", intake.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("intake power", power);
        telemetry.addData("target", turret.getTargetDeg());
        telemetry.update();
    }
}

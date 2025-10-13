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
import org.firstinspires.ftc.teamcode.subsystem.TurnTable;
import static org.firstinspires.ftc.teamcode.subsystem.TurnTable.*;
import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp(name = "Drive")
@Configurable
public class Drive extends OpMode {
    private GoBildaPinpointDriver odo;
    private DcMotorEx frontLeft, backLeft, frontRight, backRight, intake, launcher;
    private TurnTable turn;
    public static double power = 0;

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(2.3622047244, -6.6141732283, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.REVERSED,
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

        turn = new TurnTable(hardwareMap);

        PanelsConfigurables.INSTANCE.refreshClass(Drive.class);

        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        telemetry.addLine("Initialized!");
        telemetry.update();
    }

    @Override
    public void loop() {
        odo.update();
        double heading = -odo.getHeading(AngleUnit.RADIANS);

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

        if (gamepad1.dpadUpWasPressed()) power = Constants.INTAKE_ACTIVE_POWER;
        if (gamepad1.dpadDownWasPressed()) power = -Constants.INTAKE_ACTIVE_POWER;
        if (gamepad1.dpadLeftWasPressed()) power = 0;
        intake.setPower(power);

        if (gamepad1.leftBumperWasPressed()) turn.nextSlot();
        if (gamepad1.rightBumperWasPressed()) turn.prevSlot();
        if (gamepad1.aWasPressed()) turn.startLaunch(DetectedColor.GREEN);
        if (gamepad1.bWasPressed()) turn.startLaunch(DetectedColor.PURPLE);
        if (gamepad1.yWasPressed()) turn.startLaunchCurrent();

        if (gamepad1.leftStickButtonWasPressed()) launcher.setPower(1);
        if (gamepad1.rightStickButtonWasPressed()) launcher.setPower(0);

        turn.update();

        telemetry.addData("x", odo.getPosX(DistanceUnit.MM));
        telemetry.addData("y", odo.getPosY(DistanceUnit.MM));
        telemetry.addData("h", odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("slot1", turn.getLastColor(Slot.FIRST));
        telemetry.addData("slot2", turn.getLastColor(Slot.SECOND));
        telemetry.addData("slot3", turn.getLastColor(Slot.THIRD));
        telemetry.addData("velocity", intake.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("intake power", power);
        telemetry.update();
    }
}

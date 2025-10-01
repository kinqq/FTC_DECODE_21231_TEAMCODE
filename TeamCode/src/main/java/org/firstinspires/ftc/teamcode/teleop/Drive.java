package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Drive")
@Configurable
public class Drive extends OpMode {
    private GoBildaPinpointDriver odo;
    public static int num = 0;
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotorEx intake;
    private Servo turntable;
    public static double power = 0, turntablePosition = .17;

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
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightBack");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turntable = hardwareMap.get(Servo.class, "turntable");

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

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        if (gamepad1.aWasPressed()) power += 0.1;
        if (gamepad1.bWasPressed()) power -= 0.1;
        power = Range.clip(power, -1, 1);

        if (gamepad1.leftBumperWasPressed()) {
            if (turntablePosition == 0.17) turntablePosition = .56;
            else if (turntablePosition == .56) turntablePosition = .94;
            else if (turntablePosition == .94) turntablePosition = .17;
        }
        turntable.setPosition(turntablePosition);

        intake.setPower(power);

        telemetry.addData("x", odo.getPosX(DistanceUnit.MM));
        telemetry.addData("y", odo.getPosY(DistanceUnit.MM));
        telemetry.addData("h", odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("int", num);
        telemetry.addData("velocity", intake.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("intake power", power);
        telemetry.update();
    }
}

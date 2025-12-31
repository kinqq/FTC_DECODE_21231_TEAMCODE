package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.commands.MagazineCommands;

@TeleOp(name = "Not so Basic Drive")
@Configurable
public class testTeleop extends OpMode
{
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private DcMotorEx motor3;
    private DcMotorEx motor4;

    private MagazineCommands indexer;

    private int player = 0;
    private boolean eStop = false;
    private ElapsedTime eStopTimer = new ElapsedTime();

    @Override
    public void init() {
        indexer = new MagazineCommands(hardwareMap);

        motor1 = hardwareMap.get(DcMotorEx.class, "leftFront");
        motor2 = hardwareMap.get(DcMotorEx.class, "leftBack");
        motor3 = hardwareMap.get(DcMotorEx.class, "rightFront");
        motor4 = hardwareMap.get(DcMotorEx.class, "rightBack");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);

        indexer.new zero(true);
        indexer.new nextSlot();

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        eStopTimer.reset();

        telemetry.addLine("ROBOT READY");
    }

    public void loop() {

       if (player == 1 && !eStop) {
           double x = gamepad1.left_stick_x;
           double y = -gamepad1.left_stick_y;
           double rx = gamepad1.right_stick_x;

           double max;
           double frontLeftPower = y + x + rx;
           double frontRightPower = y - x - rx;
           double backLeftPower = y - x + rx;
           double backRightPower = y + x - rx;

           max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
           max = Math.max(max, Math.abs(backLeftPower));
           max = Math.max(max, Math.abs(backRightPower));

           if (max > 1.0) {
               frontLeftPower /= max;
               frontRightPower /= max;
               backLeftPower /= max;
               backRightPower /= max;
           }

           motor1.setPower(frontLeftPower);
           motor3.setPower(frontRightPower);
           motor2.setPower(backLeftPower);
           motor4.setPower(backRightPower);
       }
       else if (player == 2 && !eStop) {
            double x = gamepad2.left_stick_x;
            double y = -gamepad2.left_stick_y;
            double rx = gamepad2.right_stick_x;

            double max;
            double frontLeftPower = y + x + rx;
            double frontRightPower = y - x - rx;
            double backLeftPower = y - x + rx;
            double backRightPower = y + x - rx;

            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            motor1.setPower(frontLeftPower);
            motor3.setPower(frontRightPower);
            motor2.setPower(backLeftPower);
            motor4.setPower(backRightPower);
        }
       else if (player == 0 && !eStop) {
           motor1.setPower(0);
           motor2.setPower(0);
           motor3.setPower(0);
           motor4.setPower(0);
       }
       else if (eStop && eStopTimer.seconds() > 0.1)
       {
           motor1.setDirection(DcMotorSimple.Direction.REVERSE);
           motor2.setDirection(DcMotorSimple.Direction.FORWARD);
           motor3.setDirection(DcMotorSimple.Direction.FORWARD);
           motor4.setDirection(DcMotorSimple.Direction.REVERSE);
           motor1.setPower(0);
           motor3.setPower(0);
           motor2.setPower(0);
           motor4.setPower(0);
       }

        if (gamepad2.aWasPressed())
        {
            player++;
            player %= 3;
        }
        if (gamepad2.backWasPressed())
        {
            eStop = true;
            player = 0;
            motor1.setDirection(DcMotorSimple.Direction.FORWARD);
            motor2.setDirection(DcMotorSimple.Direction.REVERSE);
            motor3.setDirection(DcMotorSimple.Direction.REVERSE);
            motor4.setDirection(DcMotorSimple.Direction.FORWARD);
            motor1.setPower(1);
            motor3.setPower(1);
            motor2.setPower(1);
            motor4.setPower(1);
            eStopTimer.reset();
        }
        if (gamepad2.startWasPressed()) terminateOpModeNow();

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("PLAYER:");
        if (player == 0) telemetry.addLine("        NO CONTROLLER");
        if (player == 1) telemetry.addLine("        GUEST CONTROL");
        if (player == 2) telemetry.addLine("        42 CONTROL");
    }

    @Override
    public void stop() {

    }
}
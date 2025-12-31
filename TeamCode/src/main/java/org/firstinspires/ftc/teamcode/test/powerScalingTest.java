package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "powerReportingTest")
public class powerScalingTest extends OpMode
{
   public DcMotorEx motor;
   public DcMotorEx motor1;
   double motorPower = 0;

    @Override
    public void init()
    {
        motor = hardwareMap.get(DcMotorEx.class, "launcher");
        motor1 = hardwareMap.get(DcMotorEx.class, "launcher1");

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop()
    {

        if (gamepad1.aWasPressed()) motorPower = 1;
        if (gamepad1.bWasPressed()) motorPower = 0;

        if (gamepad1.xWasPressed()) motorPower = 0.5;

        if (gamepad1.rightBumperWasPressed()) motorPower += 0.1;
        if (gamepad1.leftBumperWasPressed()) motorPower -= 0.1;

        motor.setPower(motorPower);
        motor1.setPower(motorPower);

        telemetry.addData("Motor Power:", motorPower);
        telemetry.addData("Motor V", motor1.getVelocity());
        telemetry.update();

    }


}

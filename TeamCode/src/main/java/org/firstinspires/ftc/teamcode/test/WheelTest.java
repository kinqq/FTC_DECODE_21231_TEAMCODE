package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "WheelTest", group = "Test")
public class WheelTest extends OpMode
{
    DcMotor motor;

    @Override
    public void init()
    {
        motor = hardwareMap.get(DcMotor.class, "rightFront");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop()
    {
        motor.setPower(0.5);

    }

}

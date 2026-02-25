package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "Intake Test", group = "Test")
public class IntakeTest extends OpMode
{
    DcMotor motor;

    @Override
    public void init()
    {
        motor = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void loop()
    {
        if (gamepad1.a) motor.setPower(1);
        else motor.setPower(0);
    }
}

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.commands.PTOCommands;


@TeleOp (name = "ClutchTest", group = "Test")
public class ClutchTest extends OpMode
{
    Servo servo;
    Servo servo1;


    PTOCommands ptoCommands;

    @Override
    public void init()
    {
        servo = hardwareMap.get(Servo.class, "rightClutch");
        servo.setDirection(Servo.Direction.REVERSE);

        servo1 = hardwareMap.get(Servo.class, "leftClutch");
        servo1.setDirection(Servo.Direction.FORWARD);

        ptoCommands = new PTOCommands(hardwareMap);
        ptoCommands.disengageClutch();
    }

    double pos = 0.0;
    double pos1 = 0.0;

    @Override
    public void loop()
    {
        if (gamepad1.aWasPressed()) pos += 0.01;
        if (gamepad1.bWasPressed()) pos -= 0.01;

        if (gamepad1.xWasPressed()) pos1 -= 0.01;
        if (gamepad1.yWasPressed()) pos1 += 0.01;

        if (gamepad1.startWasPressed()) ptoCommands.disengageClutch();
        if (gamepad1.backWasPressed()) ptoCommands.new EngageClutch().initialize();

        if (gamepad1.guideWasPressed())
        {
            servo.setPosition(pos);
            servo1.setPosition(pos1);
        }

        telemetry.addData("Right Pos", pos);
        telemetry.addData("Left Pos", pos1);

    }

}

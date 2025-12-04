package org.firstinspires.ftc.teamcode.meet2testing.subsys;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class test extends SubsystemBase
{
    private final Servo servo;

    public test(final HardwareMap hwMap, final String name)
    {
        servo = hwMap.get(Servo.class, "turntable");
    }

    public void move(double servPos)
    {
        servo.setPosition(servPos);
    }

    public void moveBackward()
    {
        servo.setPosition(0.5);
    }


}

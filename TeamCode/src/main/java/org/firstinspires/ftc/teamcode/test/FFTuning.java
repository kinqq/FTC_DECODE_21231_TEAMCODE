package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.teleop.DriveMeet2;

import java.util.ArrayList;


@TeleOp (name = "FFTuning", group = "Tuning")
public class FFTuning extends OpMode
{
    ArrayList<String> log = new ArrayList<String>() ;
    double pow = 0;

    DcMotorEx motor;
    DcMotorEx motor1;

    @Override
    public void init()
    {
        log.add("LOG:");

        motor = hardwareMap.get(DcMotorEx.class, "launcher");
        motor1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        PanelsConfigurables.INSTANCE.refreshClass(DriveMeet2.class);
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }

    @Override
    public void loop()
    {
        if (gamepad1.leftBumperWasPressed()) pow -= 0.01;
        if (gamepad1.rightBumperWasPressed()) pow += 0.01;

        if (gamepad1.a)
        {
            motor.setPower(pow);
            motor1.setPower(pow);
        } else {
            motor.setPower(0);
            motor1.setPower(0);
        }

        if (gamepad1.bWasPressed())
        {
            log.add(
                    "Power: " + pow +
                    ", Velocity: " + -motor1.getVelocity()
            );
        }

        telemetry.addData("Power", pow);
        telemetry.addData("Vel", -motor1.getVelocity());
        telemetry.addData("LOG", log);
        telemetry.update();
    }

}

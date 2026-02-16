package org.firstinspires.ftc.teamcode.test;


import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.teleop.DriveSTATE;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Flywheel FF Tuner (Auto-Log)", group="Tuning")
public class FlywheelFeedForwardTuner extends OpMode {

    private DcMotorEx launcherMotorPrimary;
    private DcMotorEx launcherMotorSecondary;

    @Override
    public void init()
    {
        launcherMotorPrimary = hardwareMap.get(DcMotorEx.class, "launcher");
        launcherMotorSecondary = hardwareMap.get(DcMotorEx.class, "launcher1");

        launcherMotorSecondary.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotorPrimary.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotorSecondary.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PanelsConfigurables.INSTANCE.refreshClass(DriveSTATE.class);
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }

    double power = 0;
    List<String> log = new ArrayList<>();

    @Override
    public void loop()
    {


        if (gamepad1.leftBumperWasPressed()) power -= 0.01;
        if (gamepad1.rightBumperWasPressed()) power += 0.01;
        if (gamepad1.a)
        {
            launcherMotorPrimary.setPower(power);
            launcherMotorSecondary.setPower(power);
        } else
        {
            launcherMotorPrimary.setPower(0);
            launcherMotorSecondary.setPower(0);
        }

        if (gamepad1.xWasPressed()) log.add("POWER: " + power + " VELOCITY: " + -launcherMotorSecondary.getVelocity());

        telemetry.addData("Vel", -launcherMotorSecondary.getVelocity());
        telemetry.addData("Power", power);
        telemetry.addData("Log", log.toString());
    }


}
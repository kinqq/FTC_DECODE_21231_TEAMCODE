package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Subsystems.*;

@TeleOp(name = "Drive Meet 2")
public class DriveMeet2 extends OpMode
{
    private TurretSubsystem turret;
    private MagazineSubsystem magazine;

    @Override
    public void init()
    {
        magazine = new MagazineSubsystem(hardwareMap);
    }

    @Override
    public void init_loop()
    {

    }

    @Override
    public void start()
    {
        turret = new TurretSubsystem(hardwareMap);
    }

    @Override
    public void loop()
    {
        if (gamepad1.startWasPressed()) magazine.initialize();
        if (gamepad1.rightBumperWasPressed()) magazine.nextSlot();
        if (gamepad1.leftBumperWasPressed()) magazine.prevSlot();

        turret.update();

        telemetry.addData("Encoder Deg", magazine.getAnalogDeg());
        telemetry.addData("Encoder", magazine.getEncoderDeg());
        telemetry.addData("Target", magazine.getTargetDeg());
        telemetry.addData("Power", magazine.getPower());
    }
}
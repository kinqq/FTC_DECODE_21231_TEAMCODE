package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import dev.frozenmilk.sinister.loading.Pinned;

@TeleOp(name = "Drive Meet 2")
@Configurable
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


    boolean homed = false;

    @Override
    public void loop()
    {

        if (!homed && magazine.home()) homed = true;
        else if (homed) magazine.update();

        if (gamepad1.aWasPressed()) magazine.setTarget(-1346);
        if (gamepad1.bWasPressed()) magazine.setTarget(0);
        if (gamepad1.xWasPressed())
        {
            homed = false;
            magazine.setTarget(0);
        }

        turret.update();

        telemetry.addData("Encoder Deg", magazine.getAnalog());
        telemetry.addData("Encoder", magazine.getEncoder());
        telemetry.addData("Target", magazine.getTarget());
        telemetry.addData("Power", magazine.getPower());
    }


}
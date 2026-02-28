package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.subsystem.commands.PTOCommands;

@TeleOp (name = "Full PTO Test", group = "Test")
public class PTOTest extends CommandOpMode
{
    PTOCommands ptoCommands;

    @Override
    public void initialize()
    {
        ptoCommands = new PTOCommands(hardwareMap);
        ptoCommands.new DisengageClutch().initialize();
        ptoCommands.new Zero().initialize();
    }

    boolean clutchEngaged = false;
    boolean lifting = false;

    @Override
    public void run()
    {
        super.run();

        if (gamepad1.yWasPressed()) ptoCommands.new EngageClutch().initialize();
        if (gamepad1.xWasPressed()) ptoCommands.new DisengageClutch().initialize();
        if (gamepad1.guideWasPressed())
        {
            //ptoCommands.new ThrottleBack().initialize();
            ptoCommands.new ThrottleFront().initialize();
            //schedule(ptoCommands.new PositionLift());
            schedule(ptoCommands.new VelLift());
        }
        if (gamepad1.bWasPressed()) ptoCommands.new ThrottleBack().initialize();
        if (gamepad1.aWasPressed()) ptoCommands.new KillBack().initialize();

        if (gamepad1.backWasPressed()) schedule(ptoCommands.new Recover(gamepad1));

        if (gamepad2.startWasPressed())
        {
            ptoCommands.new Zero().initialize();
        }

        telemetry.addData("Engaged", clutchEngaged);
        telemetry.addData("Lifting", lifting);
        telemetry.addLine();

        telemetry.addData("Pos Error", Math.abs(ptoCommands.getRightPos() - ptoCommands.getLeftPos()));
        telemetry.addData("Left Pos", ptoCommands.getLeftPos());
        telemetry.addData("Right Pos", ptoCommands.getRightPos());
        telemetry.addLine();

        telemetry.addData("Vel Error", Math.abs(ptoCommands.getRightVel() - ptoCommands.getLeftVel()));
        telemetry.addData("Left Vel", ptoCommands.getLeftVel());
        telemetry.addData("Right Vel", ptoCommands.getRightVel());
        telemetry.addLine();

        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.commands.PTOCommands;

@TeleOp (name = "Full PTO Test", group = "Test")
public class PTOTest extends OpMode
{
    PTOCommands ptoCommands;

    @Override
    public void init()
    {
        ptoCommands = new PTOCommands(hardwareMap);
        ptoCommands.disengageClutch();
    }

    @Override
    public void loop()
    {
        if (gamepad1.yWasPressed()) ptoCommands.new EngageClutch().initialize();
        if (gamepad1.xWasPressed()) ptoCommands.disengageClutch();
        if (gamepad1.guideWasPressed())
        {
            ptoCommands.new ThrottleBack().initialize();
            ptoCommands.new ThrottleFront().initialize();
        }
        if (gamepad1.bWasPressed()) ptoCommands.new ThrottleBack().initialize();
        if (gamepad1.aWasPressed()) ptoCommands.new KillBack().initialize();

        if (gamepad1.backWasPressed()) ptoCommands.new KillFront().initialize();
        if (gamepad1.startWasPressed()) terminateOpModeNow();
    }
}

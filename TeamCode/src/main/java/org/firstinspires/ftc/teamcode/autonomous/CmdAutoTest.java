package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystem.commands.IndexerCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.util.Slot;

@Autonomous(name = "TestAuto")
public class CmdAutoTest extends CommandOpMode {

    private TurretCommands turretCommands;
    private CommandBase activeIndexerCmd;
    private Slot currentSlot = Slot.FIRST;

    @Override
    public void initialize() {
        turretCommands = new TurretCommands(hardwareMap);

        schedule(
            new SequentialCommandGroup(
                turretCommands.setTarget(30),
                new WaitCommand(2000),
                turretCommands.setTarget(-30),
                new WaitCommand(2000),
                turretCommands.setTarget(15),
                new WaitCommand(2000),
                turretCommands.setTarget(-15)
            )
        );

        telemetry.update();
    }

    @Override
    public void run() {
        super.run();


        telemetry.addData("CurrentSlot", currentSlot);
        telemetry.addData("error", turretCommands.getErrorDeg());
        telemetry.update();
    }
}
package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.commands.IndexerCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.constant.Slot;

@Autonomous(name = "TestAuto")
public class CmdAutoTest extends CommandOpMode {

    private TurretCommands turretCommands;
    private IndexerCommands indexerCmds;
    protected Follower follower;
    private CommandBase activeIndexerCmd;
    private Slot currentSlot = Slot.FIRST;

    @Override
    public void initialize() {
        turretCommands = new TurretCommands(hardwareMap);
        indexerCmds = new IndexerCommands(hardwareMap);
        indexerCmds.initializeIndexer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        PathChain path1 = follower
            .pathBuilder()
            .addPath(new BezierLine(new Pose(0,0,0), new Pose(10, 0, 0)))
            .setConstantHeadingInterpolation(0)
            .build();

        PathChain path2 = follower
            .pathBuilder()
            .addPath(new BezierLine(new Pose(10,0,0), new Pose(20, 0, 0)))
            .setConstantHeadingInterpolation(0)
            .build();

        PathChain path3 = follower
            .pathBuilder()
            .addPath(new BezierLine(new Pose(20,0,0), new Pose(30, 0, 0)))
            .setConstantHeadingInterpolation(0)
            .build();

        schedule(
            new SequentialCommandGroup(
                indexerCmds.spinToIntake(Slot.FIRST)
//                indexerCmds.spinToIntake(Slot.SECOND),
//                indexerCmds.spinToIntake(Slot.THIRD)
            )
        );

        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        indexerCmds.update();
        follower.update();

        telemetry.addData("CurrentSlot", currentSlot);
        telemetry.addData("error", indexerCmds.getTargetDeg() - indexerCmds.getCurrentDeg());
        telemetry.update();
    }
}
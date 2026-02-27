package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autonomous.commands.IndexerCommands;
import org.firstinspires.ftc.teamcode.autonomous.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.autonomous.commands.LaunchCommands;
import org.firstinspires.ftc.teamcode.autonomous.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.autonomous.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.pedropathing.PedroConstants;
import org.firstinspires.ftc.teamcode.constant.Slot;

@Autonomous(name = "TestAuto")
@Disabled
public class CmdAutoTest extends CommandOpMode {

    private TurretCommands turretCommands;
    private IndexerCommands indexerCmds;
    private IntakeCommands intakeCmds;
    private LaunchCommands launchCmds;
    private LimelightCommands ll;
    protected Follower follower;
    private CommandBase activeIndexerCmd;
    private Slot currentSlot = Slot.FIRST;

    @Override
    public void initialize() {
        turretCommands = new TurretCommands(hardwareMap);
        indexerCmds = new IndexerCommands(hardwareMap);
        intakeCmds = new IntakeCommands(hardwareMap);
        ll = new LimelightCommands(hardwareMap);
        launchCmds = new LaunchCommands(ll, indexerCmds, turretCommands);
        follower = PedroConstants.createFollower(hardwareMap);
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
                intakeCmds.intakeOn(),
                turretCommands.activateLauncher(1.0),
                turretCommands.setLaunchAngle(40),
                launchCmds.shootEachSlot(1.0)
            )
        );

        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        indexerCmds.update();
        turretCommands.update();
        follower.update();

        telemetry.addData("vel", turretCommands.launchMotor1.getVelocity());
        telemetry.update();
    }
}

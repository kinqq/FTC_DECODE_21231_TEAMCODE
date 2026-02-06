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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.commands.IndexerCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.LaunchCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.MagazineCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.constant.Slot;

@Autonomous(name = "TestAuto")
public class CmdAutoTest extends CommandOpMode {

    private TurretCommands turretCommands;
    private MagazineCommands indexerCmds;
    private IntakeCommands intakeCmds;
    private LaunchCommands launchCmds;
    LimelightCommands ll = new LimelightCommands(hardwareMap);
    protected Follower follower;
    private CommandBase activeIndexerCmd;
    private Slot currentSlot = Slot.FIRST;

    @Override
    public void initialize() {
        turretCommands = new TurretCommands(hardwareMap);
        indexerCmds = new MagazineCommands(hardwareMap);
        intakeCmds = new IntakeCommands(hardwareMap);
        ll = new LimelightCommands(hardwareMap);
        launchCmds = new LaunchCommands(ll, indexerCmds, turretCommands);
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
                intakeCmds.intakeOn(1),
                launchCmds.shootEachSlot(0.82)
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
        telemetry.addData("Servo Pos", indexerCmds.getServoPos());
        telemetry.addData("Current Target", indexerCmds.getTarget());
        telemetry.addData("Current Pos", indexerCmds.getAnalogAngle());
        telemetry.addData("isBusy", indexerCmds.isBusy());
        telemetry.addData("vel", indexerCmds.velocity);
        telemetry.addData("dist", indexerCmds.bob.getDistance(DistanceUnit.MM));
        telemetry.update();
    }
}
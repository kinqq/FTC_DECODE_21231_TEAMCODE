package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.MagazineCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.LaunchCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.constant.AllianceColor;
import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.util.GlobalState;
import org.firstinspires.ftc.teamcode.constant.Slot;
import com.pedropathing.paths.Path;

public class JustLeaveBaseCmd extends CommandOpMode {

    public Follower follower;
    public PathChain path;
    public LimelightCommands llCmds;
    public TurretSubsystem turretCmds;


    private CommandBase followPath(PathChain path) {
        return new FollowPathCommand(follower, path, true);
    }

    public Pose getStart() {return new Pose(0, 0);}
    public Pose getEnd() {return new Pose(0, 0);}
    public double getStartH() {return 0.0;}
    public double getEndH() {return 0.0;}
    public AllianceColor getAllianceColor() {return AllianceColor.RED;}

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        llCmds = new LimelightCommands(hardwareMap);
        turretCmds = new TurretSubsystem(hardwareMap);

        follower.setStartingPose(getStart());
        llCmds.start(0);

        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        path = follower.pathBuilder().addPath(
                new BezierLine(
                        getStart(),
                        getEnd()
                )
        )       .setLinearHeadingInterpolation(Math.toDegrees(getStartH()), Math.toDegrees(getEndH()))
                .build();

        schedule(followPath(path));

        turretCmds.zero();
    }

    @Override
    public void run() {
        super.run();
        follower.update();
    }

    @Override
    public void end() {
        GlobalState.teleOpStartPose = follower.getPose();
        GlobalState.lastMotif = llCmds.getLastDetectedMotif();
        GlobalState.alliance = getAllianceColor();
    }

}

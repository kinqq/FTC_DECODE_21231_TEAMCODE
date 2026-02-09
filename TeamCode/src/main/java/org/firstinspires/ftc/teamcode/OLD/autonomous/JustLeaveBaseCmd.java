package org.firstinspires.ftc.teamcode.OLD.autonomous;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.OLD.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.OLD.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.OLD.subsystem.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.OLD.constant.AllianceColor;
import org.firstinspires.ftc.teamcode.OLD.util.GlobalState;

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

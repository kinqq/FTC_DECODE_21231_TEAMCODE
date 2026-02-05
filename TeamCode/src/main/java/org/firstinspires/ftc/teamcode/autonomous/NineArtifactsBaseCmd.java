package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

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

public abstract class NineArtifactsBaseCmd extends CommandOpMode {
    private static final double INTAKE_POWER = 1.0;

    private TurretCommands turretCmds;
    private MagazineCommands indexerCmds;
    private LimelightCommands llCmds;
    private IntakeCommands intakeCmds;
    private LaunchCommands launchCmds;
    private Follower follower;
    private NineArtifactsPaths paths;

    abstract AllianceColor getAllianceColor();

    private CommandBase followPath(PathChain path) {
        return new FollowPathCommand(follower, path, true);
    }

    @Override
    public void initialize() {
        AllianceColor alliance = getAllianceColor();

        turretCmds = new TurretCommands(hardwareMap);
        indexerCmds = new MagazineCommands(hardwareMap);
        llCmds = new LimelightCommands(hardwareMap);
        intakeCmds = new IntakeCommands(hardwareMap);
        launchCmds = new LaunchCommands(llCmds, indexerCmds, turretCmds);

        turretCmds.zeroHere();
        indexerCmds.update();
        indexerCmds.hammer.setPosition(0.65);

        llCmds.start(0);

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(
            getAllianceColor() == AllianceColor.RED
                ? new Pose(116.8, 130.35, Math.toRadians(38.177))
                : new Pose(27.2, 132.0, Math.toRadians(141.823))
        );

        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        paths = new NineArtifactsPaths(follower, alliance);

        double preloadTurretTargetDeg;
        double volleyTurretTargetDeg;
        double aprilTagTurretTargetDeg;

        if (alliance == AllianceColor.RED) {
            aprilTagTurretTargetDeg = 45;
            preloadTurretTargetDeg = 3;
            volleyTurretTargetDeg = 38;
        } else {
            aprilTagTurretTargetDeg = -45;
            preloadTurretTargetDeg = -3;
            volleyTurretTargetDeg = -38;
        }

        SequentialCommandGroup shootPreload =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    turretCmds.setTarget(aprilTagTurretTargetDeg),
                    llCmds.waitForAnyMotif(),
                    followPath(paths.Path1),
                    turretCmds.activateLauncher(.82),
                    intakeCmds.intakeOn(INTAKE_POWER)
                ),
                new ParallelCommandGroup(
                    indexerCmds.setSlotColors(DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE),
                    turretCmds.setLaunchAngle(30),
                    turretCmds.setTarget(preloadTurretTargetDeg),
                    indexerCmds.lockSlot(Slot.SECOND),
                    new InstantCommand(indexerCmds::unlock)
                ),
                launchCmds.shootMotifFromDetection(.82)
            );

        SequentialCommandGroup intakeFirstRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path2),
                    turretCmds.deactivateLauncher(),
                    indexerCmds.setSlot(Slot.FIRST),
                    intakeCmds.intakeOn(INTAKE_POWER)
                ),
                new InstantCommand(() -> follower.setMaxPower(0.3)),
                new ParallelCommandGroup(
                    followPath(paths.Path3),
                    new SequentialCommandGroup(
                        indexerCmds.waitForAnyArtifact(),
                        indexerCmds.setSlot(Slot.SECOND),
                        indexerCmds.waitForAnyArtifact(),
                        indexerCmds.setSlot(Slot.THIRD),
                        indexerCmds.waitForAnyArtifact()
                    )
                ),
                new InstantCommand(() -> follower.setMaxPower(1))
            );

        SequentialCommandGroup shootFirstRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    indexerCmds.setSlotColors(DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN),
                    turretCmds.activateLauncher(.78),
                    turretCmds.setLaunchAngle(35),
                    turretCmds.setTarget(volleyTurretTargetDeg),
                    followPath(paths.Path4)
                ),
                launchCmds.shootMotifFromDetection(.78)
            );

        SequentialCommandGroup intakeSecondRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path5),
                    turretCmds.deactivateLauncher(),
                    indexerCmds.setSlot(Slot.FIRST),
                    intakeCmds.intakeOn(INTAKE_POWER)
                ),
                new InstantCommand(() -> follower.setMaxPower(0.3)),
                new ParallelCommandGroup(
                    followPath(paths.Path6),
                    new SequentialCommandGroup(
                        indexerCmds.waitForAnyArtifact(),
                        indexerCmds.setSlot(Slot.SECOND),
                        indexerCmds.waitForAnyArtifact(),
                        indexerCmds.setSlot(Slot.THIRD),
                        indexerCmds.waitForAnyArtifact()
                    )
                ),
                new InstantCommand(() -> follower.setMaxPower(1))
            );

        SequentialCommandGroup shootSecondRow =
            new SequentialCommandGroup(
                    new ParallelCommandGroup(indexerCmds.setSlotColors(DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE),
                    turretCmds.activateLauncher(.78),
                    turretCmds.setLaunchAngle(35),
                    turretCmds.setTarget(volleyTurretTargetDeg),
                    followPath(paths.Path7)
                ),
                launchCmds.shootMotifFromDetection(.78)
            );

        ParallelCommandGroup park =
            new ParallelCommandGroup(
                followPath(paths.Path8),
                intakeCmds.intakeOff(),
                turretCmds.deactivateLauncher()
            );

        schedule(
            new SequentialCommandGroup(
                shootPreload,
                intakeFirstRow,
                shootFirstRow,
                intakeSecondRow,
                shootSecondRow,
                park
            )
        );

        telemetry.addLine("Nine Artifacts " + alliance + " ready");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        follower.update();
        indexerCmds.update();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("PickLog", indexerCmds.getLastPickLog());
        telemetry.addData("Indexer Target", indexerCmds.getTarget());
        telemetry.addData("Motif Detected", llCmds.getLastDetectedMotif());
        telemetry.addData("Launch Vel.", turretCmds.launchMotor.getVelocity());
        telemetry.addData("FIRST", indexerCmds.getSlot(Slot.FIRST));
        telemetry.addData("SECOND", indexerCmds.getSlot(Slot.SECOND));
        telemetry.addData("THIRD", indexerCmds.getSlot(Slot.THIRD));
        telemetry.update();
    }

    @Override
    public void end() {
        GlobalState.teleOpStartPose = follower.getPose();
        GlobalState.lastMotif = llCmds.getLastDetectedMotif();
        GlobalState.alliance = getAllianceColor();
    }
}

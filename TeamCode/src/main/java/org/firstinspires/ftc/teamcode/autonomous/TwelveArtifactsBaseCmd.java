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
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.Draw;
import org.firstinspires.ftc.teamcode.subsystem.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.MagazineCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.LaunchCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.constant.AllianceColor;
import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.util.GlobalState;
import org.firstinspires.ftc.teamcode.constant.Slot;

public abstract class TwelveArtifactsBaseCmd extends CommandOpMode {
    private static final double INTAKE_POWER = 1.0;

    private TurretCommands turretCmds;
    private MagazineCommands indexerCmds;
    private LimelightCommands llCmds;
    private IntakeCommands intakeCmds;
    private LaunchCommands launchCmds;
    private Follower follower;
    private TwelveArtifactsPaths paths;

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
            alliance == AllianceColor.RED
                ? new Pose(116.8, 130.35, Math.toRadians(38.177))
                : new Pose(27.2, 132.0, Math.toRadians(141.823))
        );

        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        paths = new TwelveArtifactsPaths(follower, alliance);

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
                    turretCmds.activateLauncher(.85)
                ),
                new ParallelCommandGroup(
                    indexerCmds.setSlotColors(DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE),
                    turretCmds.setLaunchAngle(30),
                    turretCmds.setTarget(preloadTurretTargetDeg),
                    indexerCmds.lockSlot(Slot.FIRST),
                    new InstantCommand(indexerCmds::unlock),
                    intakeCmds.intakeOn(INTAKE_POWER)
                ),
                launchCmds.shootEachSlot(.85)
            );

        SequentialCommandGroup intakeSecondRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path2),
                    turretCmds.deactivateLauncher(),
                    indexerCmds.setSlot(Slot.FIRST),
                    intakeCmds.intakeOn(INTAKE_POWER)
                ),
                new InstantCommand(() -> follower.setMaxPower(0.35)),
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

        SequentialCommandGroup shootSecondRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    indexerCmds.setSlotColors(DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE),
                    turretCmds.activateLauncher(.85),
                    turretCmds.setLaunchAngle(35),
                    turretCmds.setTarget(volleyTurretTargetDeg),
                    followPath(paths.Path4)
                ),
                launchCmds.shootMotifFromDetection(.85)
            );

        SequentialCommandGroup intakeFirstRow =
            new SequentialCommandGroup(
                new InstantCommand(() -> follower.setMaxPower(0.35)),
                new ParallelCommandGroup(
                    followPath(paths.Path5),
                    turretCmds.deactivateLauncher(),
                    indexerCmds.setSlot(Slot.FIRST),
                    intakeCmds.intakeOn(INTAKE_POWER),
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
                    turretCmds.activateLauncher(.85),
                    turretCmds.setLaunchAngle(35),
                    turretCmds.setTarget(volleyTurretTargetDeg),
                    followPath(paths.Path6)
                ),
                launchCmds.shootMotifFromDetection(.85)
            );

        SequentialCommandGroup intakeThirdRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new InstantCommand(() -> follower.setMaxPower(0.5)),
                    followPath(paths.Path7),
                    turretCmds.deactivateLauncher(),
                    indexerCmds.setSlot(Slot.FIRST),
                    intakeCmds.intakeOn(INTAKE_POWER),
                    new SequentialCommandGroup(
                        new WaitCommand(1000),
                        indexerCmds.waitForAnyArtifact(),
                        indexerCmds.setSlot(Slot.SECOND),
                        indexerCmds.waitForAnyArtifact(),
                        indexerCmds.setSlot(Slot.THIRD),
                        indexerCmds.waitForAnyArtifact()
                    )
                ),
                new InstantCommand(() -> follower.setMaxPower(1))
            );

        SequentialCommandGroup shootThirdRow = new SequentialCommandGroup(
            new ParallelCommandGroup(
                indexerCmds.setSlotColors(DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE),
                turretCmds.activateLauncher(.85),
                turretCmds.setLaunchAngle(35),
                turretCmds.setTarget(preloadTurretTargetDeg),
                followPath(paths.Path8)
            ),
            launchCmds.shootMotifFromDetection(.85)
        );

        ParallelCommandGroup park =
            new ParallelCommandGroup(
                followPath(paths.Path9),
                intakeCmds.intakeOff(),
                turretCmds.deactivateLauncher()
            );

        schedule(
            new SequentialCommandGroup(
                shootPreload,
                intakeSecondRow,
                shootSecondRow,
                intakeFirstRow,
                shootFirstRow,
                intakeThirdRow,
                shootThirdRow,
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
        Draw.drawDebug(follower);

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("isBusy", indexerCmds.isBusy());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("PickLog", indexerCmds.getLastPickLog());
        telemetry.addData("Indexer Target", indexerCmds.getTarget());
        telemetry.addData("Motif Detected", llCmds.getLastDetectedMotif());
        telemetry.addData("Launch Vel.", turretCmds.launchMotor1.getVelocity());
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

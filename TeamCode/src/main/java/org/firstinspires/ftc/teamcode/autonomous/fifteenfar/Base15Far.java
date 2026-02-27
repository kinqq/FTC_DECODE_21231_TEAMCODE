package org.firstinspires.ftc.teamcode.autonomous.fifteenfar;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.autonomous.AllianceTransform;
import org.firstinspires.ftc.teamcode.autonomous.commands.IndexerCommands;
import org.firstinspires.ftc.teamcode.autonomous.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.autonomous.commands.LaunchCommands;
import org.firstinspires.ftc.teamcode.autonomous.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.autonomous.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.constant.AllianceColor;
import org.firstinspires.ftc.teamcode.constant.Constants;
import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.constant.Slot;
import org.firstinspires.ftc.teamcode.pedropathing.Draw;
import org.firstinspires.ftc.teamcode.pedropathing.PedroConstants;

public abstract class Base15Far extends CommandOpMode {
    private TurretCommands turretCmds;
    private IndexerCommands indexerCmds;
    private LimelightCommands llCmds;
    private IntakeCommands intakeCmds;
    private LaunchCommands launchCmds;
    private Follower follower;
    private Paths15Far paths;

    abstract AllianceColor getAllianceColor();

    private CommandBase followPath(PathChain path) {
        return new FollowPathCommand(follower, path, true);
    }

    private CommandBase revUp(double power) {
        return new InstantCommand(() -> turretCmds.activateLauncher(power).initialize());
    }

    @Override
    public void initialize() {
        AllianceColor alliance = getAllianceColor();

        turretCmds = new TurretCommands(hardwareMap);
        indexerCmds = new IndexerCommands(hardwareMap);
        llCmds = new LimelightCommands(hardwareMap);
        intakeCmds = new IntakeCommands(hardwareMap);
        launchCmds = new LaunchCommands(llCmds, indexerCmds, turretCmds);

        turretCmds.zeroHere();
        indexerCmds.update();
        indexerCmds.hammer.setPosition(Constants.MAGAZINE_HAMMER_DOWN_POS);
        indexerCmds.indexer.setPosition(Constants.MAGAZINE_SLOT_FIRST_POS);

        llCmds.start(0);

        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(new AllianceTransform(alliance).pose(88.0, 8.0, Math.toRadians(0)));


        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        paths = new Paths15Far(follower, alliance);
        double defaultTurretTargetDeg = 68, endTurretTargetDeg = 25;
        double launchAngleDeg = 40;
        double launchPower = .85;

        if (alliance == AllianceColor.BLUE) {
            defaultTurretTargetDeg *= -1;
            endTurretTargetDeg *= -1;
        }

        SequentialCommandGroup outtake = new SequentialCommandGroup(
            intakeCmds.intakeOn(-1),
            new WaitCommand(800),
            indexerCmds.hammerUp(),
            new WaitCommand(200),
            indexerCmds.hammerDown(),
            intakeCmds.intakeOn()
        );

        SequentialCommandGroup shootPreload = new SequentialCommandGroup(
            new ParallelCommandGroup(
                followPath(paths.Path1),
                indexerCmds.setSlotColors(DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE),
                turretCmds.setTarget(defaultTurretTargetDeg),
                turretCmds.setLaunchAngle(launchAngleDeg),
                revUp(launchPower),
                intakeCmds.intakeOn()
            ),
            launchCmds.shootEachSlot(launchPower)
        );

        SequentialCommandGroup intakeThirdRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path2),
                    revUp(launchPower),
                    indexerCmds.setSlot(Slot.FIRST),
                    intakeCmds.intakeOn(),
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> follower.getPathCompletion() > 0.5),
                        indexerCmds.waitForAnyArtifact(),
                        indexerCmds.setSlot(Slot.SECOND),
                        indexerCmds.waitForAnyArtifact(),
                        indexerCmds.setSlot(Slot.THIRD),
                        indexerCmds.waitForAnyArtifact()
                    )
                )
            );

        SequentialCommandGroup shootThirdRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path3),
                    indexerCmds.setSlotColors(DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE),
                    turretCmds.setLaunchAngle(launchAngleDeg),
                    turretCmds.setTarget(defaultTurretTargetDeg),
                    revUp(launchPower),
                    outtake
                ),
                launchCmds.shootEachSlot(launchPower)
            );

        SequentialCommandGroup intakeHumanPlayer =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path4),
                    revUp(0.7),
                    indexerCmds.setSlot(Slot.FIRST),
                    intakeCmds.intakeOn(),
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> follower.getPathCompletion() > 0.6),
                        indexerCmds.waitForAnyArtifact(1),
                        indexerCmds.setSlot(Slot.SECOND),
                        indexerCmds.waitForAnyArtifact(0.7),
                        indexerCmds.setSlot(Slot.THIRD),
                        indexerCmds.waitForAnyArtifact(0.7)
                    )
                )
            );

        SequentialCommandGroup shootHumanPlayer =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path5),
                    indexerCmds.setSlotColors(DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE),
                    turretCmds.setLaunchAngle(launchAngleDeg),
                    turretCmds.setTarget(defaultTurretTargetDeg),
                    revUp(launchPower),
                    outtake
                ),
                launchCmds.shootEachSlot(launchPower)
            );

        SequentialCommandGroup intakeFirstSecretTunnel =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path6),
                    revUp(0.7),
                    indexerCmds.setSlot(Slot.FIRST),
                    intakeCmds.intakeOn(),
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> follower.getPathCompletion() > 0.6),
                        indexerCmds.waitForAnyArtifact(1),
                        indexerCmds.setSlot(Slot.SECOND),
                        indexerCmds.waitForAnyArtifact(0.7),
                        indexerCmds.setSlot(Slot.THIRD),
                        indexerCmds.waitForAnyArtifact(0.7)
                    )
                )
            );

        SequentialCommandGroup shootFirstSecretTunnel =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path7),
                    indexerCmds.setSlotColors(DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN),
                    revUp(launchPower),
                    turretCmds.setLaunchAngle(launchAngleDeg),
                    turretCmds.setTarget(defaultTurretTargetDeg),
                    outtake
                ),
                launchCmds.shootEachSlot(launchPower)
            );

        SequentialCommandGroup intakeSecondSecretTunnel =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path8),
                    revUp(0.7),
                    indexerCmds.setSlot(Slot.FIRST),
                    intakeCmds.intakeOn(),
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> follower.getPathCompletion() > 0.6),
                        indexerCmds.waitForAnyArtifact(1),
                        indexerCmds.setSlot(Slot.SECOND),
                        indexerCmds.waitForAnyArtifact(0.7),
                        indexerCmds.setSlot(Slot.THIRD),
                        indexerCmds.waitForAnyArtifact(0.7)
                    )
                )
            );

        SequentialCommandGroup shootSecondSecretTunnel =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path9),
                    indexerCmds.setSlotColors(DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE),
                    revUp(launchPower),
                    turretCmds.setLaunchAngle(launchAngleDeg),
                    turretCmds.setTarget(endTurretTargetDeg),
                    outtake
                ),
                launchCmds.shootEachSlot(launchPower)
            );

        schedule(
            new SequentialCommandGroup(
                new WaitUntilCommand(this::isStarted),
                shootPreload,
                intakeThirdRow,
                shootThirdRow,
                intakeHumanPlayer,
                shootHumanPlayer,
                intakeFirstSecretTunnel,
                shootFirstSecretTunnel,
                intakeSecondSecretTunnel,
                shootSecondSecretTunnel
            )
        );
    }

    @Override
    public void run() {
        super.run();
        indexerCmds.update();
        turretCmds.update();
        follower.update();
        Draw.drawDebug(follower);

        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Launcher velocity", turretCmds.launchMotor1.getVelocity());
        telemetry.addData("Path Completion", follower.getPathCompletion());
        telemetry.addData("Motif Detected", llCmds.getLastDetectedMotif());
        telemetry.addData("Pick log", indexerCmds.getLastPickLog());
        telemetry.addData("Beam", indexerCmds.beam.isArtifactPresent());
        telemetry.update();
    }
}

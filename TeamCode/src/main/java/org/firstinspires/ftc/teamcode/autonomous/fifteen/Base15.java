package org.firstinspires.ftc.teamcode.autonomous.fifteen;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
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

public abstract class Base15 extends CommandOpMode {
    private TurretCommands turretCmds;
    private IndexerCommands indexerCmds;
    private LimelightCommands llCmds;
    private IntakeCommands intakeCmds;
    private LaunchCommands launchCmds;
    private Follower follower;
    private Paths15 paths;

    abstract AllianceColor getAllianceColor();

    private CommandBase followPath(PathChain path) {
        return new FollowPathCommand(follower, path, true);
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

        llCmds.start(0);

        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(new AllianceTransform(alliance).pose(116.8, 130.35, Math.toRadians(38.177)));

        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        paths = new Paths15(follower, alliance);

        SequentialCommandGroup shootPreload = new SequentialCommandGroup(
            new ParallelCommandGroup(
                followPath(paths.Path1),
                turretCmds.setTarget(95),
                llCmds.waitForAnyMotif(),
                turretCmds.setLaunchAngle(35),
                turretCmds.activateLauncher(0.65)
            ),
            new ParallelCommandGroup(
                indexerCmds.setSlotColors(DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE),
                turretCmds.setTarget(50),
                intakeCmds.intakeOn()
            ),
            launchCmds.shootEachSlot(0.65)
        );

        SequentialCommandGroup intakeSecondRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path2),
                    turretCmds.activateLauncher(0.5),
                    indexerCmds.setSlot(Slot.FIRST),
                    intakeCmds.intakeOn(),
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> follower.getPathCompletion() > 0.6),
                        indexerCmds.waitForAnyArtifact(),
                        indexerCmds.setSlot(Slot.SECOND),
                        indexerCmds.waitForAnyArtifact(),
                        indexerCmds.setSlot(Slot.THIRD),
                        indexerCmds.waitForAnyArtifact()
                    )
                )
            );

        SequentialCommandGroup shootSecondRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path3),
                    indexerCmds.setSlotColors(DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE),
                    turretCmds.activateLauncher(0.65),
                    turretCmds.setLaunchAngle(35),
                    turretCmds.setTarget(50),
                    new SequentialCommandGroup(
                        intakeCmds.intakeOn(-1),
                        new WaitCommand(800),
                        intakeCmds.intakeOn()
                    )
                ),
                launchCmds.shootMotifFromDetection(0.65)
            );

        SequentialCommandGroup intakeFirstRamp =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path4),
                    turretCmds.activateLauncher(0.5),
                    indexerCmds.setSlot(Slot.FIRST),
                    intakeCmds.intakeOn(),
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> follower.getPathCompletion() > 0.6),
                        indexerCmds.waitForAnyArtifact(),
                        indexerCmds.setSlot(Slot.SECOND),
                        indexerCmds.waitForAnyArtifact(),
                        indexerCmds.setSlot(Slot.THIRD),
                        indexerCmds.waitForAnyArtifact()
                    )
                )
            );

        SequentialCommandGroup shootFirstRamp =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path5),
                    indexerCmds.setSlotColors(DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE),
                    turretCmds.activateLauncher(0.65),
                    turretCmds.setLaunchAngle(35),
                    turretCmds.setTarget(50),
                    new SequentialCommandGroup(
                        intakeCmds.intakeOn(-1),
                        new WaitCommand(800),
                        intakeCmds.intakeOn()
                    )
                ),
                launchCmds.shootMotifFromDetection(0.65)
            );

        {
            SequentialCommandGroup intakeSecondRamp =
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        followPath(paths.Path4),
                        turretCmds.activateLauncher(0.5),
                        indexerCmds.setSlot(Slot.FIRST),
                        intakeCmds.intakeOn(),
                        new SequentialCommandGroup(
                            new WaitUntilCommand(() -> follower.getPathCompletion() > 0.6),
                            indexerCmds.waitForAnyArtifact(),
                            indexerCmds.setSlot(Slot.SECOND),
                            indexerCmds.waitForAnyArtifact(),
                            indexerCmds.setSlot(Slot.THIRD),
                            indexerCmds.waitForAnyArtifact()
                        )
                    )
                );

            SequentialCommandGroup shootSecondRamp =
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        followPath(paths.Path5),
                        indexerCmds.setSlotColors(DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE),
                        turretCmds.activateLauncher(0.65),
                        turretCmds.setLaunchAngle(35),
                        turretCmds.setTarget(50),
                        new SequentialCommandGroup(
                            intakeCmds.intakeOn(-1),
                            new WaitCommand(800),
                            intakeCmds.intakeOn()
                        )
                    ),
                    launchCmds.shootMotifFromDetection(0.65)
                );
        }

        SequentialCommandGroup intakeFirstRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path6),
                    turretCmds.activateLauncher(0.5),
                    indexerCmds.setSlot(Slot.FIRST),
                    intakeCmds.intakeOn(),
                    new SequentialCommandGroup(
                        indexerCmds.waitForAnyArtifact(),
                        indexerCmds.setSlot(Slot.SECOND),
                        indexerCmds.waitForAnyArtifact(),
                        indexerCmds.setSlot(Slot.THIRD),
                        indexerCmds.waitForAnyArtifact()
                    )
                )
            );

        SequentialCommandGroup shootFirstRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path7),
                    indexerCmds.setSlotColors(DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN),
                    turretCmds.activateLauncher(0.65),
                    turretCmds.setLaunchAngle(35),
                    turretCmds.setTarget(50),
                    new SequentialCommandGroup(
                        intakeCmds.intakeOn(-1),
                        new WaitCommand(800),
                        intakeCmds.intakeOn()
                    )
                ),
                launchCmds.shootMotifFromDetection(0.65)
            );

        SequentialCommandGroup intakeThirdRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path8),
                    turretCmds.activateLauncher(0.5),
                    indexerCmds.setSlot(Slot.FIRST),
                    intakeCmds.intakeOn(),
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> follower.getPathCompletion() > 0.6),
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
                    followPath(paths.Path9),
                    indexerCmds.setSlotColors(DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE),
                    turretCmds.activateLauncher(0.65),
                    turretCmds.setLaunchAngle(35),
                    turretCmds.setTarget(105),
                    new SequentialCommandGroup(
                        intakeCmds.intakeOn(-1),
                        new WaitCommand(800),
                        intakeCmds.intakeOn()
                    )
                ),
                launchCmds.shootMotifFromDetection(0.65)
            );

        schedule(
            new SequentialCommandGroup(
                new WaitUntilCommand(this::isStarted),
                shootPreload,
                intakeSecondRow,
                shootSecondRow,
                intakeFirstRamp,
                shootFirstRamp,
//                intakeSecondRamp,
//                shootSecondRamp,
                intakeFirstRow,
                shootFirstRow,
                intakeThirdRow,
                shootThirdRow
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

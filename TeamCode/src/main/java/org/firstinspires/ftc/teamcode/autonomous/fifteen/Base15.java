package org.firstinspires.ftc.teamcode.autonomous.fifteen;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
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

public abstract class Base15 extends CommandOpMode {
    private enum AutoRangeMode {
        FULL_RANGE,
        CLOSE_RANGE
    }

    private TurretCommands turretCmds;
    private IndexerCommands indexerCmds;
    private LimelightCommands llCmds;
    private IntakeCommands intakeCmds;
    private LaunchCommands launchCmds;
    private Follower follower;
    private Paths15 paths;
    private AutoRangeMode selectedRangeMode = AutoRangeMode.FULL_RANGE;

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

        llCmds.start(0);

        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(new AllianceTransform(alliance).pose(116.8, 130.35, Math.toRadians(38.177)));

        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        paths = new Paths15(follower, alliance);

        double obeliskTurretTargetDeg = 95;
        double defaultTurretTargetDeg = 50;
        double endTurretTargetDeg = 105;
        double launchAngleDeg = 35;
        double launchPower = 0.65;

        if (alliance == AllianceColor.BLUE) {
            obeliskTurretTargetDeg *= -1;
            defaultTurretTargetDeg *= -1;
            endTurretTargetDeg *= -1;
        }

        SequentialCommandGroup shootPreload = new SequentialCommandGroup(
            new ParallelCommandGroup(
                followPath(paths.Path1),
                turretCmds.setTarget(defaultTurretTargetDeg),
                turretCmds.setLaunchAngle(launchAngleDeg),
                revUp(launchPower),
                indexerCmds.setSlotColors(DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE),
                intakeCmds.intakeOn()
            ),
            launchCmds.shootEachSlot(launchPower)
        );

        SequentialCommandGroup intakeSecondRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path2),
                    revUp(0.5),
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
                    revUp(launchPower),
                    turretCmds.setLaunchAngle(launchAngleDeg),
                    turretCmds.setTarget(defaultTurretTargetDeg),
                    new SequentialCommandGroup(
                        intakeCmds.intakeOn(-1),
                        new WaitCommand(800),
                        intakeCmds.intakeOn()
                    )
                ),
                launchCmds.shootEachSlot(launchPower)
            );

        SequentialCommandGroup intakeFirstRamp =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path4),
                    revUp(0.5),
                    indexerCmds.setSlot(Slot.FIRST),
                    intakeCmds.intakeOn(),
                    turretCmds.setTarget(obeliskTurretTargetDeg),
                    llCmds.waitForAnyMotif(),
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
                    revUp(launchPower),
                    turretCmds.setLaunchAngle(launchAngleDeg),
                    turretCmds.setTarget(defaultTurretTargetDeg),
                    new SequentialCommandGroup(
                        intakeCmds.intakeOn(-1),
                        new WaitCommand(800),
                        intakeCmds.intakeOn()
                    )
                ),
                launchCmds.shootMotifFromDetection(launchPower)
            );

        SequentialCommandGroup intakeSecondRamp =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path4),
                    revUp(0.5),
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
                    revUp(launchPower),
                    turretCmds.setLaunchAngle(launchAngleDeg),
                    turretCmds.setTarget(defaultTurretTargetDeg),
                    new SequentialCommandGroup(
                        intakeCmds.intakeOn(-1),
                        new WaitCommand(800),
                        intakeCmds.intakeOn()
                    )
                ),
                launchCmds.shootMotifFromDetection(launchPower)
            );

        SequentialCommandGroup intakeFirstRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path6),
                    revUp(0.5),
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
                    revUp(launchPower),
                    turretCmds.setLaunchAngle(launchAngleDeg),
                    turretCmds.setTarget(defaultTurretTargetDeg),
                    new SequentialCommandGroup(
                        intakeCmds.intakeOn(-1),
                        new WaitCommand(800),
                        intakeCmds.intakeOn()
                    )
                ),
                launchCmds.shootMotifFromDetection(launchPower)
            );

        SequentialCommandGroup intakeThirdRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path8),
                    revUp(0.5),
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
                    revUp(launchPower),
                    turretCmds.setLaunchAngle(launchAngleDeg),
                    turretCmds.setTarget(endTurretTargetDeg),
                    new SequentialCommandGroup(
                        intakeCmds.intakeOn(-1),
                        new WaitCommand(800),
                        intakeCmds.intakeOn()
                    )
                ),
                launchCmds.shootMotifFromDetection(launchPower)
            );

        schedule(
            new SequentialCommandGroup(
                new WaitUntilCommand(this::isStarted),
                new CommandBase() {
                    private CommandBase selectedAuto;

                    @Override
                    public void initialize() {
                        if (selectedRangeMode == AutoRangeMode.FULL_RANGE) {
                            selectedAuto = new SequentialCommandGroup(
                                shootPreload,
                                intakeSecondRow,
                                shootSecondRow,
                                intakeFirstRamp,
                                shootFirstRamp,
                                intakeFirstRow,
                                shootFirstRow,
                                intakeThirdRow,
                                shootThirdRow
                            );
                        }
                        else {
                            selectedAuto = new SequentialCommandGroup(
                                shootPreload,
                                intakeSecondRow,
                                shootSecondRow,
                                intakeFirstRamp,
                                shootFirstRamp,
                                intakeSecondRamp,
                                shootSecondRamp,
                                intakeFirstRow,
                                shootFirstRow
                            );
                        }
                        selectedAuto.initialize();
                    }

                    @Override
                    public void execute() {
                        selectedAuto.execute();
                    }

                    @Override
                    public boolean isFinished() {
                        return selectedAuto.isFinished();
                    }

                    @Override
                    public void end(boolean interrupted) {
                        selectedAuto.end(interrupted);
                    }
                }
            )
        );
    }

    public void initialize_loop() {
        if (gamepad1.dpadUpWasPressed()) {
            selectedRangeMode = AutoRangeMode.FULL_RANGE;
        }
        if (gamepad1.dpadDownWasPressed()) {
            selectedRangeMode = AutoRangeMode.CLOSE_RANGE;
        }

        telemetry.addLine("Auto Range Select (init)");
        telemetry.addLine((selectedRangeMode == AutoRangeMode.FULL_RANGE ? ">" : "  ") + "[1] full range");
        telemetry.addLine((selectedRangeMode == AutoRangeMode.CLOSE_RANGE ? ">" : "  ") + "[2] close range");
        telemetry.update();
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

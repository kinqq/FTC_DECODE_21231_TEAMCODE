package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constant.Constants.HAMMER_REST;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.commands.IndexerCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.constant.AllianceColor;
import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.util.GlobalState;
import org.firstinspires.ftc.teamcode.util.MotifUtil;
import org.firstinspires.ftc.teamcode.constant.Slot;

public abstract class NineArtifactsBaseCmd extends CommandOpMode {
    protected static final double INTAKE_POWER = 1.0;

    protected TurretCommands turretCmds;
    protected IndexerCommands indexerCmds;
    protected LimelightCommands llCmds;
    protected DcMotorEx intake;
    protected Follower follower;

    protected Paths paths;

    protected abstract AllianceColor getAllianceColor();

    protected class IntakeOn extends CommandBase {
        private final double p;
        public IntakeOn(double p) { this.p = p; }
        @Override public void execute() { intake.setPower(p); }
        @Override public boolean isFinished() { return true; }
    }

    protected class IntakeOff extends CommandBase {
        @Override public void execute() { intake.setPower(0.0); }
        @Override public boolean isFinished() { return true; }
    }

    protected Slot[] pickSlotsForMotif(DetectedColor[] motifColors) {
        DetectedColor[] slotColors = new DetectedColor[] {
            indexerCmds.getSlotColor(Slot.FIRST),
            indexerCmds.getSlotColor(Slot.SECOND),
            indexerCmds.getSlotColor(Slot.THIRD)
        };
        Slot[] slotEnum = new Slot[] { Slot.FIRST, Slot.SECOND, Slot.THIRD };

        Slot[] result = new Slot[3];

        for (int i = 0; i < 3; i++) {
            DetectedColor needed = motifColors[i];
            Slot chosen = null;

            for (int j = 0; j < 3; j++) {
                if (slotColors[j] == needed) {
                    chosen = slotEnum[j];
                    slotColors[j] = DetectedColor.UNKNOWN;
                    break;
                }
            }
            result[i] = chosen;
        }

        return result;
    }

    protected CommandBase shoot(Slot slot, double power) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                indexerCmds.new SpinToShoot(slot),
                turretCmds.activateLauncher(power)
            ),
            indexerCmds.new HammerUp(),
            indexerCmds.new HammerDown()
        );
    }

    protected CommandBase shootMotifFromDetection(double power) {
        return new CommandBase() {
            private CommandBase inner;

            @Override
            public void initialize() {
                LimelightCommands.Motif motif = llCmds.getLastDetectedMotif();
                DetectedColor[] colors = MotifUtil.motifToColors(motif);

                if (colors == null || colors.length < 3) {
                    inner = new InstantCommand(() -> {});
                    inner.initialize();
                    return;
                }

                Slot[] slots = pickSlotsForMotif(colors);

                CommandBase c0 = (slots[0] != null) ? shoot(slots[0], power) : new InstantCommand(() -> {});
                CommandBase c1 = (slots[1] != null) ? shoot(slots[1], power) : new InstantCommand(() -> {});
                CommandBase c2 = (slots[2] != null) ? shoot(slots[2], power) : new InstantCommand(() -> {});

                inner = new SequentialCommandGroup(c0, c1, c2);
                inner.initialize();
            }

            @Override
            public void execute() {
                if (inner != null) inner.execute();
            }

            @Override
            public boolean isFinished() {
                return inner == null || inner.isFinished();
            }

            @Override
            public void end(boolean interrupted) {
                if (inner != null) inner.end(interrupted);
            }
        };
    }

    @Override
    public void initialize() {
        AllianceColor alliance = getAllianceColor();

        turretCmds = new TurretCommands(hardwareMap);
        indexerCmds = new IndexerCommands(hardwareMap);
        llCmds = new LimelightCommands(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(0);

        turretCmds.zeroHere();
        indexerCmds.initializeIndexer();
        indexerCmds.hammer.setPosition(HAMMER_REST);

        llCmds.start(0);

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(
            getAllianceColor() == AllianceColor.RED
                ? new Pose(116.0, 132.0, Math.toRadians(36))
                : new Pose(28.0, 132.0, Math.toRadians(144))
        );

        paths = new Paths(follower, alliance);

        double preloadTurretTargetDeg;
        double volleyTurretTargetDeg;

        if (alliance == AllianceColor.RED) {
            preloadTurretTargetDeg = 38;
            volleyTurretTargetDeg = -3;
        } else {
            preloadTurretTargetDeg = -38;
            volleyTurretTargetDeg = 3;
        }

        SequentialCommandGroup shootPreload =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    indexerCmds.new SetSlotColors(DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE),
                    turretCmds.activateLauncher(0.95),
                    turretCmds.setLaunchAngle(40),
                    turretCmds.setTarget(preloadTurretTargetDeg),
                    indexerCmds.spinToShoot(Slot.SECOND),
                    new IntakeOn(INTAKE_POWER),
                    new FollowPathCommand(follower, paths.Path1, true)
                ),
                llCmds.waitForAnyMotif(),
                shootMotifFromDetection(0.95)
            );

        SequentialCommandGroup intakeFirstRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new FollowPathCommand(follower, paths.Path2, true),
                    turretCmds.deactivateLauncher(),
                    indexerCmds.spinToIntake(Slot.FIRST),
                    new IntakeOn(INTAKE_POWER)
                ),
                new SequentialCommandGroup(
                    new FollowPathCommand(follower, paths.Path3, true),
                    indexerCmds.spinToIntake(Slot.SECOND)
                ),
                new SequentialCommandGroup(
                    new FollowPathCommand(follower, paths.Path4, true),
                    indexerCmds.spinToIntake(Slot.THIRD),
                    new InstantCommand(() -> turretCmds.activateLauncherRaw(1600))
                ),
                new FollowPathCommand(follower, paths.Path5, true)
            );

        SequentialCommandGroup shootFirstRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    indexerCmds.new SetSlotColors(DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN),
                    turretCmds.activateLauncher(.9),
                    turretCmds.setLaunchAngle(40),
                    turretCmds.setTarget(volleyTurretTargetDeg),
                    new FollowPathCommand(follower, paths.Path6, true)
                ),
                shootMotifFromDetection(.9)
            );

        SequentialCommandGroup intakeSecondRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new FollowPathCommand(follower, paths.Path7, true),
                    turretCmds.deactivateLauncher(),
                    indexerCmds.spinToIntake(Slot.FIRST),
                    new IntakeOn(INTAKE_POWER)
                ),
                new SequentialCommandGroup(
                    new FollowPathCommand(follower, paths.Path8, true),
                    indexerCmds.spinToIntake(Slot.SECOND)
                ),
                new SequentialCommandGroup(
                    new FollowPathCommand(follower, paths.Path9, true),
                    indexerCmds.spinToIntake(Slot.THIRD)
                ),
                new FollowPathCommand(follower, paths.Path10, true)
            );

        SequentialCommandGroup shootSecondRow =
            new SequentialCommandGroup(
                indexerCmds.new SetSlotColors(DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE),
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        turretCmds.activateLauncher(.9),
                        turretCmds.setLaunchAngle(40),
                        turretCmds.setTarget(volleyTurretTargetDeg)
                    ),
                    new FollowPathCommand(follower, paths.Path11, true)
                ),
                shootMotifFromDetection(.9)
            );

        ParallelCommandGroup park =
            new ParallelCommandGroup(
                new FollowPathCommand(follower, paths.Path12),
                new IntakeOff(),
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
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Indexer Target", indexerCmds.getTargetDeg());
        telemetry.addData("Motif Detected", llCmds.getLastDetectedMotif());
        telemetry.addData("Launch Vel.", turretCmds.launchMotor.getVelocity());
        telemetry.update();
    }

    @Override
    public void end() {
        GlobalState.teleOpStartPose = follower.getPose();
        GlobalState.lastMotif = llCmds.getLastDetectedMotif();
        GlobalState.alliance = getAllianceColor();
    }
}

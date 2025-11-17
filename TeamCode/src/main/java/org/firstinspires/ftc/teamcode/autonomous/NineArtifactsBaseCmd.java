package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constant.Constants.HAMMER_REST;
import static org.firstinspires.ftc.teamcode.constant.Constants.SLOT1_HOLD;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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

    public static class Paths {
        public final PathChain Path1, Path2, Path3, Path4, Path5, Path6,
            Path7, Path8, Path9, Path10, Path11, Path12;

        private static Pose mirrorPoseX(Pose redPose) {
            double fieldWidth = 144.0;
            return new Pose(
                fieldWidth - redPose.getX(),
                redPose.getY(),
                Math.PI - redPose.getHeading()
            );
        }

        private static double mirrorHeading(double h) {
            return Math.PI - h;
        }

        public Paths(Follower follower, AllianceColor allianceColor) {
            boolean red = (allianceColor == AllianceColor.RED);

            Pose p1s = new Pose(116.000, 132.000);
            Pose p1e = new Pose(92.000, 86.000);
            if (!red) { p1s = mirrorPoseX(p1s); p1e = mirrorPoseX(p1e); }
            Path1 = follower
                .pathBuilder()
                .addPath(new BezierLine(p1s, p1e))
                .setLinearHeadingInterpolation(
                    red ? Math.toRadians(36) : mirrorHeading(Math.toRadians(36)),
                    red ? Math.toRadians(90) : mirrorHeading(Math.toRadians(90))
                )
                .build();

            Pose p2s = new Pose(92.000, 86.000);
            Pose p2e = new Pose(92.000, 85.000);
            if (!red) { p2s = mirrorPoseX(p2s); p2e = mirrorPoseX(p2e); }
            Path2 = follower
                .pathBuilder()
                .addPath(new BezierLine(p2s, p2e))
                .setLinearHeadingInterpolation(
                    red ? Math.toRadians(90) : mirrorHeading(Math.toRadians(90)),
                    red ? Math.toRadians(0)  : mirrorHeading(Math.toRadians(0))
                )
                .build();

            Pose p3s = new Pose(92.000, 85.000);
            Pose p3e = new Pose(104.000, 85.000);
            if (!red) { p3s = mirrorPoseX(p3s); p3e = mirrorPoseX(p3e); }
            Path3 = follower
                .pathBuilder()
                .addPath(new BezierLine(p3s, p3e))
                .setTangentHeadingInterpolation()
                .build();

            Pose p4s = new Pose(104.000, 85.000);
            Pose p4e = new Pose(110.000, 85.000);
            if (!red) { p4s = mirrorPoseX(p4s); p4e = mirrorPoseX(p4e); }
            Path4 = follower
                .pathBuilder()
                .addPath(new BezierLine(p4s, p4e))
                .setTangentHeadingInterpolation()
                .build();

            Pose p5s = new Pose(110.000, 85.000);
            Pose p5e = new Pose(124.000, 85.000);
            if (!red) { p5s = mirrorPoseX(p5s); p5e = mirrorPoseX(p5e); }
            Path5 = follower
                .pathBuilder()
                .addPath(new BezierLine(p5s, p5e))
                .setTangentHeadingInterpolation()
                .build();

            Pose p6s = new Pose(124.000, 85.000);
            Pose p6e = new Pose(88.000, 86.000);
            if (!red) { p6s = mirrorPoseX(p6s); p6e = mirrorPoseX(p6e); }
            Path6 = follower
                .pathBuilder()
                .addPath(new BezierLine(p6s, p6e))
                .setLinearHeadingInterpolation(
                    red ? Math.toRadians(0)  : mirrorHeading(Math.toRadians(0)),
                    red ? Math.toRadians(45) : mirrorHeading(Math.toRadians(45))
                )
                .build();

            Pose p7s = new Pose(88.000, 86.000);
            Pose p7e = new Pose(94.000, 61.000);
            if (!red) { p7s = mirrorPoseX(p7s); p7e = mirrorPoseX(p7e); }
            Path7 = follower
                .pathBuilder()
                .addPath(new BezierLine(p7s, p7e))
                .setLinearHeadingInterpolation(
                    red ? Math.toRadians(45) : mirrorHeading(Math.toRadians(45)),
                    red ? Math.toRadians(0) : mirrorHeading(Math.toRadians(0))
                )
                .build();

            Pose p8s = new Pose(94.000, 61.000);
            Pose p8e = new Pose(106.000, 61.000);
            if (!red) { p8s = mirrorPoseX(p8s); p8e = mirrorPoseX(p8e); }
            Path8 = follower
                .pathBuilder()
                .addPath(new BezierLine(p8s, p8e))
                .setTangentHeadingInterpolation()
                .build();

            Pose p9s = new Pose(106.000, 61.000);
            Pose p9e = new Pose(112.000, 61.000);
            if (!red) { p9s = mirrorPoseX(p9s); p9e = mirrorPoseX(p9e); }
            Path9 = follower
                .pathBuilder()
                .addPath(new BezierLine(p9s, p9e))
                .setTangentHeadingInterpolation()
                .build();

            Pose p10s = new Pose(112.000, 61.000);
            Pose p10e = new Pose(130.000, 61.000);
            if (!red) { p10s = mirrorPoseX(p10s); p10e = mirrorPoseX(p10e); }
            Path10 = follower
                .pathBuilder()
                .addPath(new BezierLine(p10s, p10e))
                .setTangentHeadingInterpolation()
                .build();

            Pose p11a = new Pose(130.000, 61.000);
            Pose p11b = new Pose(92.000, 61.000);
            Pose p11c = new Pose(88.000, 86.000);
            if (!red) { p11a = mirrorPoseX(p11a); p11b = mirrorPoseX(p11b); p11c = mirrorPoseX(p11c); }
            Path11 = follower
                .pathBuilder()
                .addPath(new BezierCurve(p11a, p11b, p11c))
                .setLinearHeadingInterpolation(
                    red ? Math.toRadians(0)  : mirrorHeading(Math.toRadians(0)),
                    red ? Math.toRadians(45) : mirrorHeading(Math.toRadians(45))
                )
                .build();

            Pose p12s = new Pose(88.000, 86.000);
            Pose p12e = new Pose(94.000, 80.000);
            if (!red) { p12s = mirrorPoseX(p12s); p12e = mirrorPoseX(p12e); }
            Path12 = follower
                .pathBuilder()
                .addPath(new BezierLine(p12s, p12e))
                .setConstantHeadingInterpolation(
                    red ? Math.toRadians(45) : mirrorHeading(Math.toRadians(45))
                )
                .build();
        }
    }

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

        indexerCmds.indexer.setPosition(SLOT1_HOLD);
        indexerCmds.hammer.setPosition(HAMMER_REST);

        llCmds.start(0);

        follower = Constants.createFollower(hardwareMap);

        if (alliance == AllianceColor.RED) {
            follower.setStartingPose(new Pose(116.0, 132.0, Math.toRadians(36)));
        } else {
            follower.setStartingPose(new Pose(28.0, 132.0, Math.toRadians(144)));
        }

        paths = new Paths(follower, alliance);

        double preloadTurretTargetDeg;
        double volleyTurretTargetDeg;

        if (alliance == AllianceColor.RED) {
            preloadTurretTargetDeg = 38;
            volleyTurretTargetDeg = -5;
        } else {
            preloadTurretTargetDeg = -38;
            volleyTurretTargetDeg = 5;
        }

        SequentialCommandGroup shootPreload =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    indexerCmds.new SetSlotColors(DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE),
                    turretCmds.activateLauncher(0.85),
                    turretCmds.setLaunchAngle(42.5),
                    turretCmds.setTarget(preloadTurretTargetDeg),
                    new IntakeOn(INTAKE_POWER),
                    new FollowPathCommand(follower, paths.Path1, true)
                ),
                llCmds.waitForAnyMotif(),
                shootMotifFromDetection(0.8)
            );

        SequentialCommandGroup intakeFirstRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new FollowPathCommand(follower, paths.Path2, true),
                    turretCmds.deactivateLauncher(),
                    indexerCmds.spinToIntake(Slot.FIRST)
                ),
                new SequentialCommandGroup(
                    new FollowPathCommand(follower, paths.Path3, true),
                    indexerCmds.spinToIntake(Slot.SECOND)
                ),
                new SequentialCommandGroup(
                    new FollowPathCommand(follower, paths.Path4, true),
                    indexerCmds.spinToIntake(Slot.THIRD)
                ),
                new SequentialCommandGroup(
                    new FollowPathCommand(follower, paths.Path5, true)
                )
            );

        SequentialCommandGroup shootFirstRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    indexerCmds.new SetSlotColors(DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN),
                    turretCmds.activateLauncher(0.85),
                    turretCmds.setLaunchAngle(42.5),
                    turretCmds.setTarget(volleyTurretTargetDeg),
                    new FollowPathCommand(follower, paths.Path6, true)
                ),
                shootMotifFromDetection(0.85)
            );

        SequentialCommandGroup intakeSecondRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new FollowPathCommand(follower, paths.Path7, true),
                    turretCmds.deactivateLauncher(),
                    indexerCmds.spinToIntake(Slot.FIRST)
                ),
                new SequentialCommandGroup(
                    new FollowPathCommand(follower, paths.Path8, true),
                    indexerCmds.spinToIntake(Slot.SECOND)
                ),
                new SequentialCommandGroup(
                    new FollowPathCommand(follower, paths.Path9, true),
                    indexerCmds.spinToIntake(Slot.THIRD)
                ),
                new SequentialCommandGroup(
                    new FollowPathCommand(follower, paths.Path10, true)
                )
            );

        SequentialCommandGroup shootSecondRow =
            new SequentialCommandGroup(
                indexerCmds.new SetSlotColors(DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE),
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        turretCmds.activateLauncher(0.85),
                        turretCmds.setLaunchAngle(42.5),
                        turretCmds.setTarget(volleyTurretTargetDeg)
                    ),
                    new FollowPathCommand(follower, paths.Path11, true)
                ),
                shootMotifFromDetection(0.85)
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
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
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

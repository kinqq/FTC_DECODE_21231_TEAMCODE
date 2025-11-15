package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.util.Constants.HAMMER_REST;
import static org.firstinspires.ftc.teamcode.util.Constants.SLOT1_HOLD;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.commands.IndexerCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.DetectedColor;
import org.firstinspires.ftc.teamcode.util.GlobalState;
import org.firstinspires.ftc.teamcode.util.MotifUtil;
import org.firstinspires.ftc.teamcode.util.Slot;


@Autonomous(name = "NineArtifactsRed (Command)")
public class NineArtifactsRedCmd extends CommandOpMode {
    private static final double SETTLE_S = 0.5;
    private static final double INTAKE_POWER = 1.0;

    private TurretCommands turretCmds;
    private IndexerCommands indexerCmds;
    private LimelightCommands llCmds;
    private DcMotorEx intake;
    private Follower follower;

    private Paths paths;

    public static class Paths {
        public final PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12;

        public Paths(Follower follower) {
            Path1 = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(new Pose(116.000, 132.000), new Pose(92.000, 86.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(90))
                .build();

            Path2 = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(new Pose(92.000, 86.000), new Pose(92.000, 85.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

            Path3 = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(new Pose(92.000, 85.000), new Pose(104.000, 85.000))
                )
                .setTangentHeadingInterpolation()
                .build();

            Path4 = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(new Pose(104.000, 85.000), new Pose(110.000, 85.000))
                )
                .setTangentHeadingInterpolation()
                .build();

            Path5 = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(new Pose(110.000, 85.000), new Pose(124.000, 85.000))
                )
                .setTangentHeadingInterpolation()
                .build();

            Path6 = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(new Pose(124.000, 85.000), new Pose(88.000, 86.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

            Path7 = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(new Pose(88.000, 86.000), new Pose(94.000, 61.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

            Path8 = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(new Pose(94.000, 61.000), new Pose(106.000, 61.000))
                )
                .setTangentHeadingInterpolation()
                .build();

            Path9 = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(new Pose(106.000, 61.000), new Pose(112.000, 61.000))
                )
                .setTangentHeadingInterpolation()
                .build();

            Path10 = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(new Pose(112.000, 61.000), new Pose(130.000, 61.000))
                )
                .setTangentHeadingInterpolation()
                .build();

            Path11 = follower
                .pathBuilder()
                .addPath(
                    new BezierCurve(
                        new Pose(130.000, 61.000),
                        new Pose(92.000, 61.000),
                        new Pose(88.000, 86.000)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

            Path12 = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(new Pose(88.000, 86.000), new Pose(94.000, 80.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(45))
                .build();
        }
    }

    private class IntakeOn extends CommandBase {
        private final double p;

        public IntakeOn(double p) {
            this.p = p;
        }

        @Override
        public void execute() {
            intake.setPower(p);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    private class IntakeOff extends CommandBase {
        @Override
        public void execute() {
            intake.setPower(0.0);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    private Slot[] pickSlotsForMotif(DetectedColor[] motifColors) {
        DetectedColor[] slotColors = new DetectedColor[]{
            indexerCmds.getSlotColor(Slot.FIRST),
            indexerCmds.getSlotColor(Slot.SECOND),
            indexerCmds.getSlotColor(Slot.THIRD)
        };
        Slot[] slotEnum = new Slot[]{Slot.FIRST, Slot.SECOND, Slot.THIRD};

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

    private CommandBase shoot(Slot slot, double power) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                indexerCmds.new SpinToShoot(slot),
                turretCmds.activateLauncher(power)
            ),
            indexerCmds.new HammerUp(),
            indexerCmds.new HammerDown()
        );
    }

    private CommandBase shootMotifFromDetection(double power) {
        return new CommandBase() {
            private CommandBase inner;

            @Override
            public void initialize() {
                LimelightCommands.Motif motif = llCmds.getLastDetectedMotif();
                DetectedColor[] colors = MotifUtil.motifToColors(motif);

                if (colors == null || colors.length < 3) {
                    inner = new InstantCommand(() -> {
                    });
                    inner.initialize();
                    return;
                }

                Slot[] slots = pickSlotsForMotif(colors);

                CommandBase c0 = (slots[0] != null) ? shoot(slots[0], power) : new InstantCommand(() -> {
                });
                CommandBase c1 = (slots[1] != null) ? shoot(slots[1], power) : new InstantCommand(() -> {
                });
                CommandBase c2 = (slots[2] != null) ? shoot(slots[2], power) : new InstantCommand(() -> {
                });

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

    private CommandBase shootMotifFromDetection() {
        return shootMotifFromDetection(1.0);
    }


    @Override
    public void initialize() {
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

        // Pedro follower + paths
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(116.0, 132.0, Math.toRadians(36)));
        paths = new Paths(follower);

        SequentialCommandGroup shootPreload =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    indexerCmds.new SetSlotColors(DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE),
                    turretCmds.activateLauncher(0.8),
                    turretCmds.setLaunchAngle(45),
                    turretCmds.setTarget(38),
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
                    turretCmds.setTarget(-5),
                    new FollowPathCommand(follower, paths.Path6, true)
                ),

                shootMotifFromDetection(0.85)
            );

        SequentialCommandGroup intakeSecondRow = new SequentialCommandGroup(
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
                        turretCmds.setTarget(-5)
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

        telemetry.addLine("Nine Artifacts RED ready");
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
        GlobalState.alliance = AllianceColor.RED;
    }
}

package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import static org.firstinspires.ftc.teamcode.util.Constants.*;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.commands.IndexerCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.util.DetectedColor;
import org.firstinspires.ftc.teamcode.util.MotifUtil;
import org.firstinspires.ftc.teamcode.util.Slot;

@Autonomous(name = "SixArtifactsRed (Command)")
public class SixArtifactsRedCmd extends CommandOpMode {
    private static final double TURRET_TOL_DEG = 2.0;
    private static final double SETTLE_S = 0.5;
    private static final double INTAKE_POWER = 1.0;


    private Turret turret;
    private TurretCommands turretCmds;
    private IndexerCommands indexerCmds;
    private DcMotorEx intake;
    private Follower follower;
    private Paths paths;


    public static class Paths {
        public final PathChain Path1, Path2, Path3, Path4, Path5, Path6;

        public Paths(Follower follower) {
            Path1 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(116.000, 132.000), new Pose(88.000, 96.000)))
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45))
                .build();

            Path2 = follower
                .pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(88.000, 96.000),
                    new Pose(90.000, 84.000),
                    new Pose(106.500, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

            Path3 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(106.500, 84.000), new Pose(114.000, 84.000)))
                .setTangentHeadingInterpolation()
                .build();

            Path4 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(111.500, 84.000), new Pose(116.500, 84.000)))
                .setTangentHeadingInterpolation()
                .build();

            Path5 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(116.500, 84.000), new Pose(121.500, 84.000)))
                .setTangentHeadingInterpolation()
                .build();

            Path6 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(121.500, 84.000), new Pose(88.000, 96.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
        }
    }

    private class IntakeOn extends CommandBase {
        private final double p;
        public IntakeOn(double p) { this.p = p; }
        @Override public void execute() { intake.setPower(p); }
        @Override public boolean isFinished() { return true; }
    }

    private class IntakeOff extends CommandBase {
        @Override public void execute() { intake.setPower(0.0); }
        @Override public boolean isFinished() { return true; }
    }

    private CommandBase shoot(Slot slot) {
        Telemetry telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        telemetry.addLine("Shooting slot " + slot);
        telemetry.update();

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                indexerCmds.new SpinToShoot(slot),
                turretCmds.activateLauncher()
            ),
            indexerCmds.new HammerUp(),
            indexerCmds.new HammerDown(),
            indexerCmds.new SpinToIntake(slot)
        );
    }

    private CommandBase shootColor(DetectedColor color) {
        Slot slot = indexerCmds.findFirstSlotWithColor(color);
        return shoot(slot);
    }

    private CommandBase shootMotif(LimelightCommands.Motif motif) {
        DetectedColor[] colors = MotifUtil.motifToColors(motif);
        return new SequentialCommandGroup(
            shootColor(colors[0]),
            shootColor(colors[1]),
            shootColor(colors[2])
        );
    }

    @Override
    public void initialize() {
        // Subsystems
        turret = new Turret(hardwareMap);
        turretCmds = new TurretCommands(hardwareMap);
        indexerCmds = new IndexerCommands(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(0);

        indexerCmds.indexer.setPosition(SLOT1_HOLD);
        indexerCmds.hammer.setPosition(HAMMER_REST);

        // Pedro follower + paths
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(116.0, 132.0, Math.toRadians(36)));
        paths = new Paths(follower);

        // 1) Volley #1: Path1 + prime concurrently, then 3 shots
        SequentialCommandGroup volley1 =
            new SequentialCommandGroup(
                // Run path1 while waiting UNTIL (path done && turret within tol), then settle
                new ParallelCommandGroup(
                    turretCmds.activateLauncher(),
                    turretCmds.setLaunchAngle(40),
                    turretCmds.setTarget(-20),
                    new IntakeOn(INTAKE_POWER),
                    new FollowPathCommand(follower, paths.Path1, true)
                ),

                // 3 launches (FIRST, SECOND, THIRD)
                shoot(Slot.FIRST),
                shoot(Slot.SECOND),
                shoot(Slot.THIRD)
            );

        // 2) Path2: deactivate, set slot1, intake ON (already on but idempotent)
        SequentialCommandGroup collect2 =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new FollowPathCommand(follower, paths.Path2, true),
                    turretCmds.deactivateLauncher(),
                    indexerCmds.spinToIntake(Slot.FIRST),
                    new IntakeOn(INTAKE_POWER)
                )
            );

        // 3) Path3: after small dwell, set slot2, mark slot1=PURPLE
        SequentialCommandGroup collect3 =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new WaitCommand((long)(SETTLE_S * 1000)),
                    new FollowPathCommand(follower, paths.Path3, true),
                    indexerCmds.spinToIntake(Slot.SECOND)
                )
            );

        // 4) Path4: dwell, set slot3, mark slot2=PURPLE
        SequentialCommandGroup collect4 =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new WaitCommand((long)(SETTLE_S * 1000)),
                    new FollowPathCommand(follower, paths.Path4, true),
                    indexerCmds.spinToIntake(Slot.THIRD)
                )
            );

        // 5) Path5: mark slot3=GREEN
        SequentialCommandGroup collect5 =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new WaitCommand((long)(SETTLE_S * 1000)),
                    new FollowPathCommand(follower, paths.Path5, true)
                )
            );

        // 6) Path6 (return) & prime volley #2, then fire A/B/C
        SequentialCommandGroup volley2 =
            new SequentialCommandGroup(
                indexerCmds.new SetSlotColors(DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN),
                // prime while driving Path6
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        turretCmds.activateLauncher(),
                        turretCmds.setLaunchAngle(40),
                        turretCmds.setTarget(-4)
                    ),
                    new FollowPathCommand(follower, paths.Path6, true)
                ),

                // deterministic A/B/C = FIRST/SECOND/THIRD
                shoot(Slot.FIRST),
                shoot(Slot.SECOND),
                shoot(Slot.THIRD),

                // done
                turretCmds.deactivateLauncher()
            );

        // ---------- Schedule full auto ----------
        schedule(
            new SequentialCommandGroup(
                volley1,
                collect2,
                collect3,
                collect4,
                collect5,
                volley2
            )
        );

        // --- TELEMETRY BEFORE START ---
        telemetry.addLine("SixArtifactsRed (Command) ready");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run(); // runs the scheduler

        // Keep Pedro follower updating every loop
        follower.update();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Turret err", "%.1f", turret.getErrorDeg());
        telemetry.update();
    }
}

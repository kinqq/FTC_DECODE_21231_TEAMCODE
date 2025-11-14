package org.firstinspires.ftc.teamcode.autonomous;

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
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Magazine;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.util.DetectedColor;
import org.firstinspires.ftc.teamcode.util.Slot;

import java.util.function.BooleanSupplier;

@Autonomous(name = "SixArtifactsRed (Command)")
public class SixArtifactsRedCmd extends CommandOpMode {

    // ---------- Tunables ----------
    private static final double TURRET_TOL_DEG = 2.0;
    private static final double PATH_TIMEOUT_S = 4.0;
    private static final double SETTLE_S       = 1.5;         // dwell between actions
    private static final double INTAKE_POWER   = 1.0;

    // ---------- HW / Subsystems ----------
    private Turret turret;
    private TurretCommands turretCmd;
    private Magazine mag;
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

    // ---------- Small utility commands ----------
    /** Wait until condition is true OR timeout seconds elapse (timeout<=0 -> no timeout). */
    private static class WaitUntil extends CommandBase {
        private final BooleanSupplier condition;
        private final long timeoutMs;
        private long start;

        public WaitUntil(BooleanSupplier condition) {
            this(condition, 0);
        }
        public WaitUntil(BooleanSupplier condition, double timeoutSec) {
            this.condition = condition;
            this.timeoutMs = timeoutSec > 0 ? (long) (timeoutSec * 1000) : 0;
        }
        @Override public void initialize() { start = System.currentTimeMillis(); }
        @Override public boolean isFinished() {
            if (condition.getAsBoolean()) return true;
            if (timeoutMs > 0 && System.currentTimeMillis() - start >= timeoutMs) return true;
            return false;
        }
    }

    /** Intake ON/OFF commands */
    private class IntakeOn extends CommandBase {
        private final double p;
        public IntakeOn(double p) { this.p = p; }
        @Override public void initialize() { intake.setPower(p); }
        @Override public boolean isFinished() { return true; }
    }
    private class IntakeOff extends CommandBase {
        @Override public void initialize() { intake.setPower(0.0); }
        @Override public boolean isFinished() { return true; }
    }

    /** Magazine: set active slot if idle */
    private class MagSetSlot extends CommandBase {
        private final Slot slot;
        public MagSetSlot(Slot slot) { this.slot = slot; }
        @Override public void initialize() {
            if (!mag.isBusy()) mag.setSlot(slot);
        }
        @Override public boolean isFinished() { return true; }
    }

    /** Magazine: start launch for a slot (non-blocking), then wait-until-done + settle */
    private class MagLaunch extends SequentialCommandGroup {
        public MagLaunch(Slot slot) {
            super(
                new InstantCommand(() -> { if (!mag.isBusy()) mag.tryStartLaunch(slot); }),
                new WaitUntil(() -> !mag.isBusy()),
                new WaitCommand((long)(SETTLE_S * 1000))
            );
        }
    }

    /** Mark a single slot’s color without changing others (Instant). */
    private class MagMarkColor extends CommandBase {
        private final Slot slot;
        private final DetectedColor color;
        public MagMarkColor(Slot slot, DetectedColor color) {
            this.slot  = slot;
            this.color = color;
        }
        @Override public void initialize() {
            DetectedColor c1 = mag.getLastColor(Slot.FIRST);
            DetectedColor c2 = mag.getLastColor(Slot.SECOND);
            DetectedColor c3 = mag.getLastColor(Slot.THIRD);
            switch (slot) {
                case FIRST:  c1 = color; break;
                case SECOND: c2 = color; break;
                case THIRD:  c3 = color; break;
            }
            mag.setColors(c1, c2, c3);
        }
        @Override public boolean isFinished() { return true; }
    }

    private class MagMarkColors extends CommandBase {
        private final DetectedColor color1, color2, color3;
        public MagMarkColors(DetectedColor color1, DetectedColor color2, DetectedColor color3) {
            this.color1 = color1;
            this.color2 = color2;
            this.color3 = color3;
        }
        @Override public void initialize() {
            mag.setColors(color1, color2, color3);
        }
        @Override public boolean isFinished() { return true; }
    }

    // ---------- OpMode lifecycle ----------
    @Override
    public void initialize() {
        // Subsystems
        turret = new Turret(hardwareMap);
        turretCmd = new TurretCommands(hardwareMap);
        mag    = new Magazine(hardwareMap);
        mag.init();
        mag.setColors(DetectedColor.UNKNOWN, DetectedColor.UNKNOWN, DetectedColor.UNKNOWN);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(0);

        // Pedro follower + paths
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(116.0, 132.0, Math.toRadians(36)));
        paths = new Paths(follower);

        // --- TELEMETRY BEFORE START ---
        telemetry.addLine("SixArtifactsRed (Command) ready");
        telemetry.update();

        // ---------- DEFINE THE FULL AUTO SEQUENCE ----------
        // Helpers
        Runnable turretUpdate = () -> turret.update();

        // Condition helpers
        BooleanSupplier path1Done = () -> !follower.isBusy();
        BooleanSupplier turretPrimed = () -> Math.abs(turret.getErrorDeg()) <= TURRET_TOL_DEG;

        // 1) Volley #1: Path1 + prime concurrently, then 3 shots
        SequentialCommandGroup volley1 =
            new SequentialCommandGroup(
                // Activate & prime turret immediately (launcher on, angle, target)
                turretCmd.activateLauncher(),
                turretCmd.setLaunchAngle(40),
                turretCmd.setTarget(-6),

                // intake on as we go
                new IntakeOn(INTAKE_POWER),

                // Run path1 while waiting UNTIL (path done && turret within tol), then settle
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new WaitUntil(() -> path1Done.getAsBoolean() && turretPrimed.getAsBoolean(), PATH_TIMEOUT_S),
                        new WaitCommand((long)(SETTLE_S * 1000))
                    ),
                    new FollowPathCommand(follower, paths.Path1, true),
                    new RunCommand(turretUpdate) // keep PID updated while waiting
                ),

                // 3 launches (FIRST, SECOND, THIRD)
                new MagLaunch(Slot.FIRST),
                new MagLaunch(Slot.SECOND),
                new MagLaunch(Slot.THIRD)
            );

        // 2) Path2: deactivate, set slot1, intake ON (already on but idempotent)
        SequentialCommandGroup collect2 =
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new WaitUntil(() -> !follower.isBusy(), PATH_TIMEOUT_S),
                        new WaitCommand((long)(SETTLE_S * 1000))
                    ),
                    new FollowPathCommand(follower, paths.Path2, true),
                    turretCmd.deactivateLauncher(),
                    new MagSetSlot(Slot.FIRST),
                    new IntakeOn(INTAKE_POWER),
                    new RunCommand(turretUpdate)
                )
            );

        // 3) Path3: after small dwell, set slot2, mark slot1=PURPLE
        SequentialCommandGroup collect3 =
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new WaitCommand((long)(SETTLE_S * 1000)), // short dwell
                        new WaitUntil(() -> !follower.isBusy())
                    ),
                    new FollowPathCommand(follower, paths.Path3, true),
                    new MagSetSlot(Slot.SECOND),
                    new RunCommand(turretUpdate)
                )
            );

        // 4) Path4: dwell, set slot3, mark slot2=PURPLE
        SequentialCommandGroup collect4 =
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new WaitCommand((long)(SETTLE_S * 1000)),
                        new WaitUntil(() -> !follower.isBusy())
                    ),
                    new FollowPathCommand(follower, paths.Path4, true),
                    new MagSetSlot(Slot.THIRD),
                    new RunCommand(turretUpdate)
                )
            );

        // 5) Path5: mark slot3=GREEN
        SequentialCommandGroup collect5 =
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new WaitUntil(() -> !follower.isBusy()),
                        new WaitCommand((long)(SETTLE_S * 1000))
                    ),
                    new FollowPathCommand(follower, paths.Path5, true),
                    new RunCommand(turretUpdate)
                )
            );

        // 6) Path6 (return) & prime volley #2, then fire A/B/C
        SequentialCommandGroup volley2 =
            new SequentialCommandGroup(
                new MagMarkColors(DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN),
                // prime while driving Path6
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new WaitUntil(() -> !follower.isBusy() && turretPrimed.getAsBoolean(), PATH_TIMEOUT_S),
                        new WaitCommand((long)(SETTLE_S * 1000)),
                        new IntakeOff() // stop intake before firing
                    ),
                    new SequentialCommandGroup(
                        turretCmd.activateLauncher(),
                        turretCmd.setLaunchAngle(40),
                        turretCmd.setTarget(-4)
                    ),
                    new FollowPathCommand(follower, paths.Path6, true),
                    new RunCommand(turretUpdate)
                ),

                // deterministic A/B/C = FIRST/SECOND/THIRD
                new MagLaunch(Slot.FIRST),
                new MagLaunch(Slot.SECOND),
                new MagLaunch(Slot.THIRD),

                // done
                turretCmd.deactivateLauncher()
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
    }

    @Override
    public void run() {
        super.run(); // runs the scheduler

        // Keep Pedro follower updating every loop
        follower.update();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Mag busy", mag.isBusy());
        telemetry.addData("Turret err", "%.1f", turret.getErrorDeg());
        telemetry.update();
    }
}

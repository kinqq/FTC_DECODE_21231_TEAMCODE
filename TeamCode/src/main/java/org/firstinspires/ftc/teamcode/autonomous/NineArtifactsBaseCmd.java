package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.MagazineCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.constant.AllianceColor;
import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.util.GlobalState;
import org.firstinspires.ftc.teamcode.util.MotifUtil;
import org.firstinspires.ftc.teamcode.constant.Slot;

public abstract class NineArtifactsBaseCmd extends CommandOpMode {
    private static final double INTAKE_POWER = 1.0;

    private TurretCommands turretCmds;
    private MagazineCommands indexerCmds;
    private LimelightCommands llCmds;
    private DcMotorEx intake;
    private Follower follower;

    private Paths paths;


    abstract AllianceColor getAllianceColor();

    public CommandBase intakeOn(double power) { return new InstantCommand(() -> intake.setPower(power)); }
    public CommandBase intakeOff() { return new InstantCommand(() -> intake.setPower(0.0)); }

    private CommandBase shoot(Slot slot, double power) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                turretCmds.activateLauncher(power),
                indexerCmds.setSlot(slot)
            ),
            indexerCmds.hammerUp(),
            indexerCmds.setColor(slot, DetectedColor.UNKNOWN)
        );
    }

    private CommandBase shootMotifFromDetection(double power) {
        LimelightCommands.Motif motif = llCmds.getLastDetectedMotif();
        DetectedColor[] motifTranslated = MotifUtil.motifToColors(motif);

        MagazineCommands.Target[] t = indexerCmds.pickTargetsForMotif(motifTranslated);
        if (t == null) return new InstantCommand(() -> {});

        return new SequentialCommandGroup(
            new InstantCommand(() -> indexerCmds.lockTo(t[0].pos)),
            shoot(t[0].slot, power),

            new InstantCommand(() -> indexerCmds.lockTo(t[1].pos)),
            shoot(t[1].slot, power),

            new InstantCommand(() -> indexerCmds.lockTo(t[2].pos)),
            shoot(t[2].slot, power),

            new InstantCommand(indexerCmds::unlock)
        );
    }

    private CommandBase followPath(PathChain path) {
        return new FollowPathCommand(follower, path, true);
    }

    @Override
    public void initialize() {
        AllianceColor alliance = getAllianceColor();

        turretCmds = new TurretCommands(hardwareMap);
        indexerCmds = new MagazineCommands(hardwareMap);
        llCmds = new LimelightCommands(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(0);

        turretCmds.zeroHere();
        indexerCmds.update();
        indexerCmds.hammer.setPosition(0.65);

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
            preloadTurretTargetDeg = -35;
            volleyTurretTargetDeg = -53;
        } else {
            preloadTurretTargetDeg = -38;
            volleyTurretTargetDeg = 3;
        }

        SequentialCommandGroup shootPreload =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    llCmds.waitForAnyMotif(),
                    followPath(paths.Path1),
                    turretCmds.activateLauncher(0.93)
                ),
                new ParallelCommandGroup(
                    indexerCmds.setSlotColors(DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE),
                    turretCmds.setLaunchAngle(30),
                    turretCmds.setTarget(alliance == AllianceColor.RED ? preloadTurretTargetDeg : -preloadTurretTargetDeg),
                    indexerCmds.setSlot(Slot.SECOND),
                    intakeOn(INTAKE_POWER)
                ),
                shootMotifFromDetection(0.893)
            );

        SequentialCommandGroup intakeFirstRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path2),
                    turretCmds.deactivateLauncher(),
                    indexerCmds.setSlot(Slot.FIRST),
                    intakeOn(INTAKE_POWER)
                ),
                new SequentialCommandGroup(
                    followPath(paths.Path3),
                    indexerCmds.setSlot(Slot.SECOND)
                ),
                new SequentialCommandGroup(
                    followPath(paths.Path4),
                    indexerCmds.setSlot(Slot.THIRD),
                    new InstantCommand(() -> turretCmds.activateLauncherRaw(1600))
                ),
                followPath(paths.Path5)
            );

        SequentialCommandGroup shootFirstRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    indexerCmds.setSlotColors(DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN),
                    turretCmds.activateLauncher(.88),
                    turretCmds.setLaunchAngle(30),
                        turretCmds.setTarget(alliance == AllianceColor.RED ? preloadTurretTargetDeg : -preloadTurretTargetDeg),
                    followPath(paths.Path6)
                ),
                shootMotifFromDetection(.88)
            );

        SequentialCommandGroup intakeSecondRow =
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followPath(paths.Path7),
                    turretCmds.deactivateLauncher(),
                    indexerCmds.setSlot(Slot.FIRST),
                    intakeOn(INTAKE_POWER)
                ),
                new SequentialCommandGroup(
                    followPath(paths.Path8),
                    indexerCmds.setSlot(Slot.SECOND)
                ),
                new SequentialCommandGroup(
                    followPath(paths.Path9),
                    indexerCmds.setSlot(Slot.THIRD)
                ),
                followPath(paths.Path10)
            );

        SequentialCommandGroup shootSecondRow =
            new SequentialCommandGroup(
                indexerCmds.setSlotColors(DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE),
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        turretCmds.activateLauncher(.88),
                        turretCmds.setLaunchAngle(0.22),
                        turretCmds.setTarget(alliance == AllianceColor.RED ? preloadTurretTargetDeg : -preloadTurretTargetDeg)
                    ),
                    followPath(paths.Path11)
                ),
                shootMotifFromDetection(.88)
            );

        ParallelCommandGroup park =
            new ParallelCommandGroup(
                followPath(paths.Path12),
                intakeOff(),
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
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
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

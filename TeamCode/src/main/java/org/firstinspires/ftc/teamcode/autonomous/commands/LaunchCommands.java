package org.firstinspires.ftc.teamcode.autonomous.commands;

import static org.firstinspires.ftc.teamcode.constant.Constants.LAUNCH_INTER_SHOT_WAIT_MS;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.constant.Motif;
import org.firstinspires.ftc.teamcode.constant.Slot;
import org.firstinspires.ftc.teamcode.util.MotifUtil;

public class LaunchCommands {
    LimelightCommands llCmds;
    IndexerCommands indexerCmds;
    TurretCommands turretCmds;

    public LaunchCommands(LimelightCommands llCmds, IndexerCommands indexerCmds, TurretCommands turretCmds) {
        this.llCmds = llCmds;
        this.indexerCmds = indexerCmds;
        this.turretCmds = turretCmds;
    }

    public CommandBase shoot(IndexerCommands.Target target, double power) {
        return new SequentialCommandGroup(
            turretCmds.activateLauncher(power),
            indexerCmds.lockSlot(target.pos),
            indexerCmds.hammerUp(),
            indexerCmds.setColor(target.slot, DetectedColor.UNKNOWN)
        );
    }

    public CommandBase shoot(Slot slot, double power) {
        return new SequentialCommandGroup(
            turretCmds.activateLauncher(power),
            indexerCmds.setSlot(slot),
            indexerCmds.hammerUp(),
            indexerCmds.setColor(slot, DetectedColor.UNKNOWN)
        );
    }

    public CommandBase shootEachSlot(double power) {
        return new SequentialCommandGroup(
            indexerCmds.setSlot(Slot.FIRST),
            shoot(Slot.FIRST, power),
            waitForLaunch(),
            shoot(Slot.SECOND, power),
            waitForLaunch(),
            shoot(Slot.THIRD, power),
            waitForLaunch(),
            new WaitCommand(100),
            indexerCmds.hammerDown()
        );
    }

    public CommandBase waitForLaunch() {
        return new ParallelRaceGroup(
                new WaitCommand(200),
                new WaitUntilCommand(() -> !indexerCmds.beam.isArtifactPresent())
            );
    }

//    public CommandBase shootEachSlot(double p1, double p2, double p3, int t1, int t2, int t3) {
//        return new SequentialCommandGroup(
//            indexerCmds.lockSlot(Slot.FIRST),
//            new WaitCommand(t1),
//            shoot(Slot.FIRST, p1),
//            new WaitCommand(t2),
//            shoot(Slot.SECOND, p2),
//            new WaitCommand(t3),
//            shoot(Slot.THIRD, p3),
//            new WaitCommand(500),
//            indexerCmds.hammerDown(),
//            new InstantCommand(indexerCmds::unlock)
//        );
//    }

    public CommandBase shootMotifFromDetection(double power) {
        return new CommandBase() {
            private CommandBase inner;

            @Override
            public void initialize() {
                Motif motif = llCmds.getLastDetectedMotif();
                DetectedColor[] motifTranslated = MotifUtil.motifToColors(motif);

                if (motifTranslated == null) {
                    motifTranslated = MotifUtil.motifToColors(Motif.PPG);
                }

                IndexerCommands.Target[] t = indexerCmds.pickTargetsForMotif(motifTranslated);
                if (t == null) {
                    inner = new InstantCommand(() -> {});
                    inner.initialize();
                    return;
                }

                inner = new SequentialCommandGroup(
                    indexerCmds.lockSlot(t[0].pos),
                    shoot(t[0], power),
                    waitForLaunch(),
                    shoot(t[1], power),
                    waitForLaunch(),
                    shoot(t[2], power),
                    waitForLaunch(),
                    new WaitCommand(100),
                    indexerCmds.hammerDown(),
                    new InstantCommand(indexerCmds::unlock)
                );
                inner.initialize();
            }

            @Override
            public void execute() {
                inner.execute();
            }

            @Override
            public boolean isFinished() {
                return inner.isFinished();
            }

            @Override
            public void end(boolean interrupted) {
                inner.end(interrupted);
            }
        };
    }
}

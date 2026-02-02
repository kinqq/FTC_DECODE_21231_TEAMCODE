package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.constant.Slot;
import org.firstinspires.ftc.teamcode.util.MotifUtil;

public class LaunchCommands {
    LimelightCommands llCmds;
    MagazineCommands indexerCmds;
    TurretCommands turretCmds;

    public LaunchCommands(LimelightCommands llCmds, MagazineCommands indexerCmds, TurretCommands turretCmds) {
        this.llCmds = llCmds;
        this.indexerCmds = indexerCmds;
        this.turretCmds = turretCmds;
    }

    public CommandBase shoot(MagazineCommands.Target target, double power) {
        return new SequentialCommandGroup(
            turretCmds.activateLauncher(power),
            indexerCmds.lockSlot(target.pos),
            indexerCmds.setColor(target.slot, DetectedColor.UNKNOWN)
        );
    }

    public CommandBase shoot(Slot slot, double power) {
        return new SequentialCommandGroup(
            turretCmds.activateLauncher(power),
            indexerCmds.setSlot(slot)
        );
    }

    public CommandBase shootMotifFromDetection(double power) {
        return new CommandBase() {
            private CommandBase inner;

            @Override
            public void initialize() {
                LimelightCommands.Motif motif = llCmds.getLastDetectedMotif();
                DetectedColor[] motifTranslated = MotifUtil.motifToColors(motif);

                if (motifTranslated == null) {
                    inner = new InstantCommand(() -> {});
                    inner.initialize();
                    return;
                }
                MagazineCommands.Target[] t = indexerCmds.pickTargetsForMotif(motifTranslated);
                if (t == null) {
                    inner = new InstantCommand(() -> {});
                    inner.initialize();
                    return;
                }
                inner = new SequentialCommandGroup(
                    indexerCmds.lockSlot(t[0].pos),
                    indexerCmds.hammerUp(),
                    shoot(t[0], power),
                    new WaitCommand(300),
                    shoot(t[1], power),
                    new WaitCommand(300),
                    shoot(t[2], power),
                    new WaitCommand(500),
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

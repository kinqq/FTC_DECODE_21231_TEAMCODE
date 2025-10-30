package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Turret;

public class TurretCommands {

    /** Move turret to a specific absolute angle */
    public static class SetTarget extends CommandBase {
        private final Turret turret;
        private final double targetDeg;

        public SetTarget(Turret turret, double targetDeg) {
            this.turret = turret;
            this.targetDeg = targetDeg;
        }

        @Override
        public void initialize() {
            turret.setTarget(targetDeg);
        }

        @Override
        public void execute() {
            turret.update();
        }

        @Override
        public boolean isFinished() {
            return Math.abs(turret.getErrorDeg()) < 1.0;
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) turret.stop();
        }
    }

    /** Increment turret by delta angle */
    public static class AdjustTarget extends CommandBase {
        private final Turret turret;
        private final double deltaDeg;

        public AdjustTarget(Turret turret, double deltaDeg) {
            this.turret = turret;
            this.deltaDeg = deltaDeg;
        }

        @Override
        public void initialize() {
            turret.adjustTarget(deltaDeg);
        }

        @Override
        public void execute() {
            turret.update();
        }

        @Override
        public boolean isFinished() {
            return Math.abs(turret.getErrorDeg()) < 1.0;
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) turret.stop();
        }
    }

    /** Reset turret to zero position */
    public static class Zero extends CommandBase {
        private final Turret turret;
        public Zero(Turret turret) {
            this.turret = turret;
        }
        @Override
        public void initialize() {
            turret.zeroHere();
        }
        @Override
        public boolean isFinished() {
            return true;
        }
    }

    /** Set launch angle (20–50 degrees) */
    public static class SetLaunchAngle extends CommandBase {
        private final Turret turret;
        private final double angle;
        public SetLaunchAngle(Turret turret, double angle) {
            this.turret = turret;
            this.angle = angle;
        }
        @Override
        public void initialize() {
            turret.setLaunchAngle(angle);
        }
        @Override
        public boolean isFinished() {
            return true;
        }
    }

    /** Turn on launcher motor */
    public static class ActivateLauncher extends CommandBase {
        private final Turret turret;
        public ActivateLauncher(Turret turret) {
            this.turret = turret;
        }
        @Override
        public void initialize() {
            turret.activateLauncher();
        }
        @Override
        public boolean isFinished() {
            return true;
        }
    }

    /** Turn off launcher motor */
    public static class DeactivateLauncher extends CommandBase {
        private final Turret turret;
        public DeactivateLauncher(Turret turret) {
            this.turret = turret;
        }
        @Override
        public void initialize() {
            turret.deactivateLauncher();
        }
        @Override
        public boolean isFinished() {
            return true;
        }
    }

    /** Toggle launcher state */
    public static class ToggleLauncher extends CommandBase {
        private final Turret turret;
        public ToggleLauncher(Turret turret) {
            this.turret = turret;
        }
        @Override
        public void initialize() {
            turret.toggleLauncher();
        }
        @Override
        public boolean isFinished() {
            return true;
        }
    }
}

package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

public class TurretCommands {
    Turret turret;

    public TurretCommands(HardwareMap hwMap) {
        turret = new Turret(hwMap);
    }

    /** Move turret to a specific absolute angle */
    public class SetTarget extends CommandBase {
        private final double targetDeg;

        public SetTarget(double targetDeg) {
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
    public CommandBase setTarget(double targetDeg) { return new SetTarget(targetDeg); }

    /** Increment turret by delta angle */
    public class AdjustTarget extends CommandBase {
        private final double deltaDeg;

        public AdjustTarget(double deltaDeg) {
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
    public CommandBase AdjustTarget(double deltaDeg) { return new AdjustTarget(deltaDeg); }

    /** Reset turret to zero position */
    public class Zero extends CommandBase {
        @Override
        public void initialize() {
            turret.zeroHere();
        }
        @Override
        public boolean isFinished() {
            return true;
        }
    }
    public CommandBase zero() { return new Zero(); }


    /** Set launch angle (20–50 degrees) */
    public class SetLaunchAngle extends CommandBase {
        private final double angle;
        public SetLaunchAngle(double angle) {
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
    public CommandBase setLaunchAngle(double angle) { return new SetLaunchAngle(angle); }


    /** Turn on launcher motor */
    public class ActivateLauncher extends CommandBase {
        ElapsedTime timer;
        @Override
        public void initialize() {
            turret.activateLauncher();
            timer = new ElapsedTime();
            timer.reset();
        }
        @Override
        public boolean isFinished() {
            return turret.launchMotor.getVelocity() > 1850 || timer.seconds() > 3;
        }
    }
    public CommandBase activateLauncher() { return new ActivateLauncher(); }

    /** Turn off launcher motor */
    public class DeactivateLauncher extends CommandBase {
        @Override
        public void initialize() {
            turret.deactivateLauncher();
        }
        @Override
        public boolean isFinished() {
            return true;
        }
    }
    public CommandBase deactivateLauncher() { return new DeactivateLauncher(); }

    /** Toggle launcher state */
    public class ToggleLauncher extends CommandBase {
        @Override
        public void initialize() {
            turret.toggleLauncher();
        }
        @Override
        public boolean isFinished() {
            return true;
        }
    }
    public CommandBase toggleLauncher() { return new ToggleLauncher(); }
}

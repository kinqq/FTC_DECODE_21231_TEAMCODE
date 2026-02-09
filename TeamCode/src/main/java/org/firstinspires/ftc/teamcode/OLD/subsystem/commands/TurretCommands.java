package org.firstinspires.ftc.teamcode.OLD.subsystem.commands;

import static org.firstinspires.ftc.teamcode.OLD.constant.LauncherPIDFConstants.d;
import static org.firstinspires.ftc.teamcode.OLD.constant.LauncherPIDFConstants.f;
import static org.firstinspires.ftc.teamcode.OLD.constant.LauncherPIDFConstants.i;
import static org.firstinspires.ftc.teamcode.OLD.constant.LauncherPIDFConstants.p;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;

public class TurretCommands {

    public final DcMotorEx turretMotor;
    public final DcMotorEx launchMotor;
    public final DcMotorEx launchMotor1;
    private final ServoImplEx launchAngle;

    private static final double MOTOR_TO_TURRET_GEAR_RATIO = 5.6111111111;
    private static final double TICKS_PER_REV = 537.7;

    private static final double TURRET_POWER = 1;

    public double targetTurretDeg = 0.0;
    private double offset = 0.0;
    private double angle = 0.0;

    public TurretCommands(HardwareMap hwMap) {
        turretMotor = hwMap.get(DcMotorEx.class, "turret");
        launchMotor = hwMap.get(DcMotorEx.class, "launcher");
        launchMotor1 = hwMap.get(DcMotorEx.class, "launcher1");
        launchAngle = hwMap.get(ServoImplEx.class, "launchAngle");

        launchAngle.setPwmRange(new PwmControl.PwmRange(500, 2500));

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(0);

        launchMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launchMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
        launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        launchMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        launchMotor.setPower(0);
        launchMotor1.setPower(0);

        targetTurretDeg = getAngleDeg();
    }

    private double getMotorAngleDeg() {
        return turretMotor.getCurrentPosition() * (360.0 / TICKS_PER_REV);
    }

    public double getAngleDeg() {
        return getMotorAngleDeg() / MOTOR_TO_TURRET_GEAR_RATIO;
    }

    public double getErrorDeg() {
        return targetTurretDeg - getAngleDeg();
    }


    private void updateMotorTarget() {
        double turretDeg = Range.clip(targetTurretDeg + offset, -90, 135);

        double motorDeg = turretDeg * MOTOR_TO_TURRET_GEAR_RATIO;
        int targetTicks = (int) Math.round(motorDeg * TICKS_PER_REV / 360.0);

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_POWER);
    }

    public void setTargetDeg(double turretDeg) {
        targetTurretDeg = turretDeg;
        updateMotorTarget();
    }

    public void adjustTargetDeg(double deltaTurretDeg) {
        offset += deltaTurretDeg;
        updateMotorTarget();
    }

    public void zeroHere() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(0);
        targetTurretDeg = 0.0;
        offset = 0.0;
    }

    public void setLaunchAngleDeg(double angleDeg) {
        angle = Range.clip(angleDeg, 15, 60);

        double pos = 0.815 - 0.003 * angle;
        launchAngle.setPosition(Range.clip(pos, 0.0, 1.0));
    }

    public void activateLauncherRaw(double velocity) {
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setPower(1.0);
        launchMotor1.setPower(1);
        launchMotor.setVelocity(velocity);
        launchMotor1.setVelocity(velocity);

    }
    public void activateLauncherRaw() {
        activateLauncherRaw(1800);
    }

    public void deactivateLauncherRaw() {
        launchMotor.setPower(0.0);
        launchMotor1.setPower(0);
        launchMotor.setVelocity(0);
        launchMotor1.setVelocity(0);

    }

    public void toggleLauncherRaw() {
        if (launchMotor.getPower() != 0) deactivateLauncherRaw();
        else activateLauncherRaw();
    }

    public void stopTurretMotor() {
        turretMotor.setPower(0);
    }

    // --- Commands ---

    public class SetTarget extends CommandBase {
        private final double targetDeg;
        private boolean started = false;

        public SetTarget(double targetDeg) { this.targetDeg = targetDeg; }

        @Override
        public void initialize() {
            started = false;
        }

        @Override
        public void execute() {
            if (!started) {
                setTargetDeg(targetDeg);
                started = true;
            }
        }

        @Override
        public boolean isFinished() {
            if (!started) return false;
            return !turretMotor.isBusy() || Math.abs(getErrorDeg()) < 1.0;
        }

        @Override
        public void end(boolean interrupted) {
            stopTurretMotor();
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public CommandBase setTarget(double targetDeg) { return new SetTarget(targetDeg); }


    public class AdjustTarget extends CommandBase {
        private final double deltaDeg;
        private boolean started = false;

        public AdjustTarget(double deltaDeg) { this.deltaDeg = deltaDeg; }

        @Override
        public void initialize() {
            started = false;
        }

        @Override
        public void execute() {
            if (!started) {
                adjustTargetDeg(deltaDeg);
                started = true;
            }
        }

        @Override
        public boolean isFinished() {
            if (!started) return false;
            return !turretMotor.isBusy() || Math.abs(getErrorDeg()) < 1.0;
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                stopTurretMotor();
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }
    public CommandBase AdjustTarget(double deltaDeg) { return new AdjustTarget(deltaDeg); }

    public class Zero extends CommandBase {
        @Override public void initialize() { zeroHere(); }
        @Override public boolean isFinished() { return true; }
    }
    public CommandBase zero() { return new Zero(); }

    public class SetLaunchAngle extends CommandBase {
        private final double angle;
        private boolean started = false;
        public SetLaunchAngle(double angle) { this.angle = angle; }

        @Override public void execute() {
            if (!started) {
                setLaunchAngleDeg(angle);
                started = true;
            }
        }
        @Override public boolean isFinished() { return true; }
    }
    public CommandBase setLaunchAngle(double angle) { return new SetLaunchAngle(angle); }

    public class ActivateLauncher extends CommandBase {
        private final ElapsedTime timer = new ElapsedTime();
        private boolean started = false;
        private final double power;

        public ActivateLauncher(double power) {
            this.power = power;
        }

        @Override
        public void initialize() {
            started = false;
        }

        @Override
        public void execute() {
            if (!started) {
                activateLauncherRaw(1800 * power);
                timer.reset();
                started = true;
            }
        }

        @Override
        public boolean isFinished() {
            double target = 1800 * power;
            boolean motor0AtSpeed = Math.abs(launchMotor.getVelocity() - target) < 30;
            boolean motor1AtSpeed = Math.abs(launchMotor1.getVelocity() - target) < 30;
            return (motor0AtSpeed && motor1AtSpeed) || timer.seconds() > 3.0;
        }
    }
    public CommandBase activateLauncher() { return new ActivateLauncher(1.0); }
    public CommandBase activateLauncher(double power) { return new ActivateLauncher(power); }

    public class DeactivateLauncher extends CommandBase {
        @Override public void initialize() { deactivateLauncherRaw(); }
        @Override public boolean isFinished() { return true; }
    }
    public CommandBase deactivateLauncher() { return new DeactivateLauncher(); }

    public class ToggleLauncher extends CommandBase {
        @Override public void initialize() { toggleLauncherRaw(); }
        @Override public boolean isFinished() { return true; }
    }
    public CommandBase toggleLauncher() { return new ToggleLauncher(); }
}

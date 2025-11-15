package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TurretCommands {

    public final DcMotorEx turretMotor;
    public final DcMotorEx launchMotor;
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
        launchAngle = hwMap.get(ServoImplEx.class, "launchAngle");

        launchAngle.setPwmRange(new PwmControl.PwmRange(500, 2500));

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(0);

        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setPower(0);

        targetTurretDeg = getAngleDeg();
    }

    // --- 내부 계산 함수들 ---

    private double getMotorAngleDeg() {
        return turretMotor.getCurrentPosition() * (360.0 / TICKS_PER_REV);
    }

    public double getAngleDeg() {
        return getMotorAngleDeg() / MOTOR_TO_TURRET_GEAR_RATIO;
    }

    public double getErrorDeg() {
        return targetTurretDeg - getAngleDeg();
    }

    /** targetTurretDeg / offset을 이용해 모터 RUN_TO_POSITION 세팅 */
    private void updateMotorTarget() {
        double turretDeg = Range.clip(targetTurretDeg + offset, -90, 135);

        double motorDeg = turretDeg * MOTOR_TO_TURRET_GEAR_RATIO;
        int targetTicks = (int) Math.round(motorDeg * TICKS_PER_REV / 360.0);

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_POWER);
    }

    // --- 퍼블릭 API ---

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
        double pos = -0.0025581395 * angle + 0.968372093;
        launchAngle.setPosition(Range.clip(pos, 0.0, 1.0));
    }

    public void activateLauncherRaw(double velocity) {
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setPower(1.0);
        launchMotor.setVelocity(velocity);
    }
    public void activateLauncherRaw() {
        activateLauncherRaw(1900);
    }

    public void deactivateLauncherRaw() {
        launchMotor.setPower(0.0);
        launchMotor.setVelocity(0);
    }

    public void toggleLauncherRaw() {
        if (launchMotor.getPower() != 0) deactivateLauncherRaw();
        else activateLauncherRaw();
    }

    public void stopTurretMotor() {
        turretMotor.setPower(0);
    }

    // --- Commands ---

    /** 특정 각도로 이동시키는 Command: execute 첫 루프에서만 setTargetDeg 적용 */
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

    /** 현재 target에서 상대적으로 이동 (execute 첫 루프에서만 adjust) */
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
        private double power = 1.0;

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
                activateLauncherRaw(1900 * power);
                timer.reset();
                started = true;
            }
        }

        @Override
        public boolean isFinished() {
            if (!started) return false;
            return launchMotor.getVelocity() > 1900 * power - 40 || timer.seconds() > 3.0;
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

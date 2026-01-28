package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constant.AllianceColor;

public class TurretSubsystem
{
    private final DcMotorEx motor;
    private final ServoImplEx launchAngle;
    private final DcMotorEx launcher;
    private final DcMotorEx launcher1;

    private double lineToGoal;
    private double angle = 0.18;
    private double vel = 1900;
    private double offset = 0;
    private double target;

    protected final int TURRET_LEFT = -82;
    protected final int TURRET_RIGHT = 45;

    public TurretSubsystem(HardwareMap hwMap)
    {
        motor = hwMap.get(DcMotorEx.class, "turret");
        launchAngle = hwMap.get(ServoImplEx.class, "launchAngle");
        launcher = hwMap.get(DcMotorEx.class, "launcher");
        launcher1 = hwMap.get(DcMotorEx.class, "launcher1");

        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher1.setDirection(DcMotorSimple.Direction.REVERSE);

        launchAngle.setDirection(Servo.Direction.REVERSE);
    }


    public void update(boolean autoPower, boolean autoAim, double power, double hoodAngle, double offset, AllianceColor alliance, double robotX, double robotY, double robotHeading)
    {
        this.offset = offset;
        this.angle = hoodAngle;

        if (!autoPower)
        {
            vel = 1800 * power;
            launchAngle.setPosition(hoodAngle);
        } else
        {
            vel = autoPower(lineToGoal);
            double angle = autoHood(lineToGoal);
            launchAngle.setPosition(angle);
        }

        double deg;

        double goalX = alliance == AllianceColor.RED ? 145 : 0;
        double goalY = 145;
        double distX = goalX - robotX;
        double distY = goalY - robotY;
        lineToGoal = Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));

        if (autoAim) {
            deg = Math.toDegrees(Math.atan2(distY, distX));
            deg -= Math.toDegrees(robotHeading);
            deg += offset;
            deg = AngleUnit.normalizeDegrees(deg);
            deg = Range.clip(deg, TURRET_LEFT, TURRET_RIGHT);
        } else deg = offset;

        double target = deg * 5.6111111111;
        int fTarget = (int) Math.round(target * 384.5 / 360.0);
        this.target = fTarget;

        motor.setTargetPosition(fTarget);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);

    }

    public double autoPower(double goalDist) {
        return Range.clip((0.004 * Math.pow(goalDist, 3)) - (0.926154 * Math.pow(goalDist, 2)) + (73.28462 * goalDist) - 626.23077, 0, 1800);
    }

    public double autoHood(double goalDist) {
        return Range.clip(((1.66203e-7) * Math.pow(goalDist, 3)) - (0.0000423232 * Math.pow(goalDist, 2)) + (0.00462337 * goalDist) + 0.0877451, 0, 0.301);
    }


    public void zero() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getDist() {
        return lineToGoal;
    }

    public double getLaunchAngle() {
        return angle;
    }

    public int getPos() {
        return motor.getCurrentPosition();
    }

    public double getTarget() {
        return target;
    }

    public double getOffset() {
        return offset;
    }

    public double getVel() {
        return launcher1.getVelocity();
    }

    public double getExpectedVel() {
        return vel;
    }

    public class spinUp extends CommandBase {
        ElapsedTime timer = new ElapsedTime();

        public spinUp() {
        }

        @Override
        public void initialize() {
            launcher.setPower(1);
            launcher1.setPower(1);
            launcher1.setVelocity(vel);
            timer.reset();
        }

        public boolean isFinished() {return motorToSpeed() || timer.seconds() > 5;}
    }

    public class spinUpRaw extends CommandBase {
        ElapsedTime timer = new ElapsedTime();
        private final double velocity;

        public spinUpRaw(double velocity) {
            this.velocity = velocity;
        }

        @Override
        public void initialize() {
            launcher.setPower(1);
            launcher1.setPower(1);
            launcher1.setVelocity(velocity);
            timer.reset();
        }

        public boolean isFinished() {return motorToSpeed() || timer.seconds() > 5;}
    }

    public class spinDown extends CommandBase {
        public spinDown() {}

        @Override
        public void initialize() {
            launcher.setPower(0);
            launcher1.setPower(0);
        }

        @Override
        public boolean isFinished() {return true;}
    }

    public boolean motorToSpeed() {
        return Math.abs(launcher1.getVelocity() - vel) < 50;
    }
}

package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.constant.LauncherPIDFConstants.p;
import static org.firstinspires.ftc.teamcode.constant.LauncherPIDFConstants.i;
import static org.firstinspires.ftc.teamcode.constant.LauncherPIDFConstants.d;
import static org.firstinspires.ftc.teamcode.constant.LauncherPIDFConstants.f;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constant.AllianceColor;

public class TurretCommands
{
    private final DcMotorEx turretMotor;
    private final DcMotorEx launcherMotorPrimary;
    private final DcMotorEx launcherMotorSecondary;

    private final ServoImplEx hoodAngleServo;

    private final VoltageSensor voltage;

    PController pid;
    private double kF;


    private double turretTargetDeg;
    private double distToGoal;

    private double hoodAngle;
    private double flywheelVelocity;
    private double turretOffset;

    private boolean spinFlywheel;

    private boolean overrideAim;
    private double overrideDegree;


    public TurretCommands(HardwareMap hwMap)
    {
        turretMotor = hwMap.get(DcMotorEx.class, "turret");
        launcherMotorPrimary = hwMap.get(DcMotorEx.class, "launcher");
        launcherMotorSecondary = hwMap.get(DcMotorEx.class, "launcher1");
        hoodAngleServo = hwMap.get(ServoImplEx.class, "launchAngle");
        voltage = hwMap.get(VoltageSensor.class, "Control Hub");

        pid = new PController(p);
        pid.setP(p);
        kF = f;

        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherMotorPrimary.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherMotorSecondary.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p, i, d, f);
        launcherMotorPrimary.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        launcherMotorSecondary.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        hoodAngleServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        hoodAngleServo.setDirection(Servo.Direction.REVERSE);

        turretTargetDeg = 0;
        distToGoal = 0;
        hoodAngle = 0.18;
        flywheelVelocity = 1220;
        turretOffset = 0;
        overrideAim = false;
        overrideDegree = 0;
        spinFlywheel = false;
    }

    public void start()
    {
        hoodAngleServo.setPosition(0.18);
        turretMotor.setTargetPosition(180);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1);
    }

    public void update(
            boolean autoPower,
            boolean autoAim,
            double velocity,
            double angle,
            double offset,
            double idlePower,
            double robotX,
            double robotY,
            double robotH,
            AllianceColor alliance)
    {
        turretOffset = offset;

        double targetX = alliance == AllianceColor.RED ? 144 : 0;
        double targetY = 144;
        double xDistance = targetX - robotX;
        double yDistance = targetY - robotY;
        distToGoal = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));

        if (autoPower)
        {
            flywheelVelocity = calculateAutoVelocity(distToGoal);
            hoodAngle = calculateAutoHoodAngle(distToGoal);
        } else
        {
            flywheelVelocity = velocity;
            hoodAngle = angle;
        }

        double targetDegree;
        if (!overrideAim)
        {
            if (autoAim) {
                targetDegree = Math.toDegrees(Math.atan2(yDistance, xDistance));
                targetDegree -= Math.toDegrees(robotH);
                targetDegree += offset;
                targetDegree = AngleUnit.normalizeDegrees(targetDegree);
                targetDegree = Range.clip(targetDegree, -185, 185);
            } else targetDegree = offset;
        } else
        {
            targetDegree = overrideDegree;
        }

        turretTargetDeg = targetDegree;

        int targetReal = (int) Math.round((targetDegree * 5.6111111111) * 384.5 / 360.0);
        turretMotor.setTargetPosition(targetReal);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1);
        hoodAngleServo.setPosition(Range.clip(hoodAngle, 0.19, 0.85));

        double pidResult = pid.calculate(-launcherMotorSecondary.getVelocity(), flywheelVelocity);
        double ff = kF * flywheelVelocity;

        double power = Range.clip(pidResult + ff, -1, 1);

        if (spinFlywheel) {
            launcherMotorPrimary.setPower(power);
            launcherMotorSecondary.setPower(power);
        } else
        {
            launcherMotorPrimary.setPower(idlePower);
            launcherMotorSecondary.setPower(idlePower);
        }
    }

    public void zero()
    {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void overrideTurret(double targetDegree)
    {
        overrideAim = true;
        overrideDegree = targetDegree;
    }

    public void spinUpToVelocity()
    {
        spinFlywheel = true;
    }

    public void deactivateLauncher()
    {
        spinFlywheel = false;
        launcherMotorPrimary.setPower(0);
        launcherMotorSecondary.setPower(0);
    }

    public double calculateAutoVelocity(double goalDist)
    {
        return Range.clip(((-0.0102645) * Math.pow(goalDist, 2)) + (7.49441 * goalDist) + 679.88926, 0, 2200);
    }

    public double calculateAutoHoodAngle(double goalDist)
    {
        return Range.clip(((-0.00000688104) * Math.pow(goalDist, 2)) + (0.00816319 * goalDist) - 0.145203, 0, 1);
    }

    public double getRealVelocity()
    {
        return -launcherMotorSecondary.getVelocity();
    }

    public double getExpectedVelocity()
    {
        return flywheelVelocity;
    }

    public double getDistToGoal()
    {
        return distToGoal;
    }

    public boolean flywheelAtExpectedSpeed()
    {
        return Math.abs((-launcherMotorSecondary.getVelocity()) - flywheelVelocity) < 20;
    }

    public class SpinUp extends CommandBase
    {
        ElapsedTime timer = new ElapsedTime();
        boolean toSpeedLast;


        @Override
        public void initialize()
        {
            spinUpToVelocity();
        }

        @Override
        public boolean isFinished()
        {
            return flywheelAtExpectedSpeed();
        }
    }

    public class SpinDown extends CommandBase
    {
        ElapsedTime timer = new ElapsedTime();

        @Override
        public void initialize()
        {
            deactivateLauncher();
        }

        @Override
        public boolean isFinished()
        {
            return true;
        }
    }
}

package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.LauncherPIDF.p;
import static org.firstinspires.ftc.teamcode.Constants.LauncherPIDF.i;
import static org.firstinspires.ftc.teamcode.Constants.LauncherPIDF.d;
import static org.firstinspires.ftc.teamcode.Constants.LauncherPIDF.f;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.sun.tools.javac.jvm.Code;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.OLD.constant.AllianceColor;

public class TurretCommands
{
    private final DcMotorEx turretMotor;
    private final DcMotorEx launcherMotorPrimary;
    private final DcMotorEx launcherMotorSecondary;

    private final Servo hoodAngleServo;

    private double turretTargetDeg;
    private double distToGoal;

    private double hoodAngle;
    private double flywheelVelocity;
    private double turretOffset;

    private boolean overrideAim;
    private double overrideDegree;

    public TurretCommands(HardwareMap hwMap)
    {
        turretMotor = hwMap.get(DcMotorEx.class, "turret");
        launcherMotorPrimary = hwMap.get(DcMotorEx.class, "launcher");
        launcherMotorSecondary = hwMap.get(DcMotorEx.class, "launcher1");
        hoodAngleServo = hwMap.get(Servo.class, "launchAngle");

        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherMotorPrimary.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherMotorSecondary.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p, i, d, f);
        launcherMotorPrimary.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        launcherMotorSecondary.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        hoodAngleServo.setDirection(Servo.Direction.REVERSE);

        turretTargetDeg = 0;
        distToGoal = 0;
        hoodAngle = 0.18;
        flywheelVelocity = 1220;
        turretOffset = 0;
        overrideAim = false;
        overrideDegree = 0;
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
            double robotX,
            double robotY,
            double robotH,
            AllianceColor alliance)
    {
        turretOffset = offset;

        double targetX = alliance == AllianceColor.RED ? 145 : 0;
        double targetY = 145;
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
            hoodAngleServo.setPosition(hoodAngle);
        }

        double targetDegree;
        if (!overrideAim)
        {
            if (autoAim) {
                targetDegree = Math.toDegrees(Math.atan2(yDistance, xDistance));
                targetDegree -= Math.toDegrees(robotH);
                targetDegree += offset;
                targetDegree = AngleUnit.normalizeDegrees(targetDegree);
                targetDegree = Range.clip(targetDegree, -82, 75);
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
        hoodAngleServo.setPosition(hoodAngle);
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
        launcherMotorPrimary.setVelocity(flywheelVelocity);
        launcherMotorSecondary.setVelocity(flywheelVelocity);
    }

    public void deactivateLauncher()
    {
        launcherMotorPrimary.setPower(0);
        launcherMotorSecondary.setPower(0);
    }

    public double calculateAutoVelocity(double goalDist)
    {
        return Range.clip((0.004 * Math.pow(goalDist, 3)) - (0.926154 * Math.pow(goalDist, 2)) + (73.28462 * goalDist) - 626.23077, 0, 1800);
    }

    public double calculateAutoHoodAngle(double goalDist)
    {
        return Range.clip(((1.66203e-7) * Math.pow(goalDist, 3)) - (0.0000423232 * Math.pow(goalDist, 2)) + (0.00462337 * goalDist) + 0.0877451, 0, 0.301);
    }

    public boolean flywheelAtExpectedSpeed()
    {
        return Math.abs(launcherMotorPrimary.getVelocity() - flywheelVelocity) < 50;
    }

    public class SpinUp extends CommandBase
    {
        ElapsedTime timer = new ElapsedTime();

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

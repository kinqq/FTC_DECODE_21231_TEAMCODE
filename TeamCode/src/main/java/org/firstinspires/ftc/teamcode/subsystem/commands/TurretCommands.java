package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.constant.AutoPowerData.Distances;
import static org.firstinspires.ftc.teamcode.constant.AutoPowerData.HoodAngles;
import static org.firstinspires.ftc.teamcode.constant.AutoPowerData.Velocities;
import static org.firstinspires.ftc.teamcode.constant.LauncherPIDFConstants.p;
import static org.firstinspires.ftc.teamcode.constant.LauncherPIDFConstants.i;
import static org.firstinspires.ftc.teamcode.constant.LauncherPIDFConstants.d;
import static org.firstinspires.ftc.teamcode.constant.LauncherPIDFConstants.f;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constant.AllianceColor;
import org.firstinspires.ftc.teamcode.constant.ShooterConstants;

public class TurretCommands
{
    private final DcMotorEx turretMotor;
    private final DcMotorEx launcherMotorPrimary;
    private final DcMotorEx launcherMotorSecondary;

    private final ServoImplEx hoodAngleServo;

    private final VoltageSensor voltage;

    PController pid;

    private double turretTargetDeg;
    private double distToGoal;

    private double hoodAngle;
    private double flywheelVelocity;
    private double turretOffset;

    private boolean spinFlywheel;

    public TurretCommands(HardwareMap hwMap)
    {
        turretMotor = hwMap.get(DcMotorEx.class, "turret");
        launcherMotorPrimary = hwMap.get(DcMotorEx.class, "launcher");
        launcherMotorSecondary = hwMap.get(DcMotorEx.class, "launcher1");
        hoodAngleServo = hwMap.get(ServoImplEx.class, "launchAngle");
        voltage = hwMap.get(VoltageSensor.class, "Control Hub");

        pid = new PController(p);
        pid.setP(p);

        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        Pose red = new Pose(145, 145);
        Pose blue = red.mirror();
        double targetX = alliance == AllianceColor.RED ? red.getX() : blue.getX();
        double targetY = 145;
        double xDistance = targetX - robotX;
        double yDistance = targetY - robotY;
        distToGoal = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));

        if (autoPower)
        {
//            flywheelVelocity = calculateAutoVelocity(distToGoal);
//            hoodAngle = calculateAutoHoodAngle(distToGoal);

            double[] power = interpolatePower(distToGoal);
            flywheelVelocity = power[0];
            hoodAngle = power[1];
        } else
        {
            flywheelVelocity = velocity;
            hoodAngle = angle;
        }

        double targetDegree;
        if (autoAim) {
            targetDegree = Math.toDegrees(Math.atan2(yDistance, xDistance));
            targetDegree -= Math.toDegrees(robotH);
            targetDegree += offset;
            targetDegree = AngleUnit.normalizeDegrees(targetDegree);
            targetDegree = Range.clip(targetDegree, -185, 185);
        } else targetDegree = offset;

        turretTargetDeg = targetDegree;

        int targetReal = (int) Math.round((targetDegree * 5.6111111111) * 384.5 / 360.0);
        turretMotor.setTargetPosition(targetReal);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1);
        hoodAngleServo.setPosition(Range.clip(hoodAngle, 0.19, 0.98));

        double kS = 0.07630;
        double kV = 0.00037270;

        double pidResult = pid.calculate(-launcherMotorSecondary.getVelocity(), flywheelVelocity);
        double ff = kS + kV * flywheelVelocity;

        double batt = voltage.getVoltage();   // REV Hub
        double cmd = (pidResult + ff) * (12.74 / batt);

        double power = Range.clip(cmd, -1, 1);

        if (spinFlywheel) {
            launcherMotorPrimary.setPower(power);
            launcherMotorSecondary.setPower(power);
        } else
        {
            launcherMotorPrimary.setPower(idlePower);
            launcherMotorSecondary.setPower(idlePower);
        }
    }

    public void updateWithPhysics(
            boolean autoPower,
            boolean autoAim,
            double velocity,
            double angle,
            double offset,
            double idlePower,
            Follower follower,
            AllianceColor alliance)
    {
        Pose goalPose = alliance == AllianceColor.RED ? ShooterConstants.GOAL_POS_RED : ShooterConstants.GOAL_POS_BLUE;
        double distX = goalPose.getX() - follower.getPose().getX();
        double distY = goalPose.getY() - follower.getPose().getY();
        double angleToGoal = Math.atan2(distY, distX);
        //angleToGoal -= follower.getPose().getHeading();
        //angleToGoal = AngleUnit.normalizeRadians(angleToGoal);

        double distToGoal = Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));
        this.distToGoal = distToGoal;

        Vector robotToGoalVector = new Vector(distToGoal, angleToGoal);

        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        double y = ShooterConstants.SCORE_HEIGHT;
        double a = ShooterConstants.SCORE_ANGLE;

        //double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), ShooterConstants.HOOD_MIN_ANGLE,
        //        ShooterConstants.HOOD_MAX_ANGLE);
        double hoodAngle = MathFunctions.clamp(getRadFromServoTicks(interpolatePower(robotToGoalVector.getMagnitude())[1]), ShooterConstants.HOOD_MIN_ANGLE,
                ShooterConstants.HOOD_MAX_ANGLE);

        double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

        Vector robotVelocity = follower.getVelocity();

        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();

        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        hoodAngle = MathFunctions.clamp(Math.atan2(vz, nvr), ShooterConstants.HOOD_MIN_ANGLE,
                ShooterConstants.HOOD_MAX_ANGLE);
        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));
        double kExit = 1300.0 / 800;  // start with ~2.24 from your observation
        flywheelSpeed *= kExit;

        double turretVelCompOffset = Math.atan2(perpendicularComponent, ivr);
        double turretAngle = Math.toDegrees(follower.getPose().getHeading() - robotToGoalVector.getTheta() + turretVelCompOffset);

        if (autoPower)
        {
            flywheelVelocity = flywheelTicksFromVelocity(flywheelSpeed);
            this.hoodAngle = getServoTicksFromDeg(Math.toDegrees(hoodAngle));
        } else
        {
            flywheelVelocity = velocity;
            this.hoodAngle = angle;
        }

        int turretTarget;

        if (autoAim) {
            turretAngle = AngleUnit.normalizeDegrees(turretAngle);
            turretAngle = Range.clip(turretAngle, -185, 185);

            turretTarget = (int) Math.round((turretAngle * 5.6111111111) * 384.5 / 360.0);
        }
        else turretTarget = (int) Math.round((offset * 5.6111111111) * 384.5 / 360.0);

        turretMotor.setTargetPosition(-turretTarget);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1);

        hoodAngleServo.setPosition(Range.clip(this.hoodAngle, 0.19, 0.98));

        double kS = 0.07439;
        double kV = 0.00035733;

        double pidResult = pid.calculate(-launcherMotorSecondary.getVelocity(), flywheelVelocity);
        double ff = Math.signum(flywheelVelocity) * kS + kV * flywheelVelocity;

        double batt = voltage.getVoltage();
        double cmd = (pidResult + ff) * (12.74 / batt);

        double power = Range.clip(cmd, -1, 1);

        if (spinFlywheel)
        {
            launcherMotorPrimary.setPower(power);
            launcherMotorSecondary.setPower(power);
        } else
        {
            launcherMotorPrimary.setPower(idlePower);
            launcherMotorSecondary.setPower(idlePower);
        }
    }

    public double flywheelTicksFromVelocity(double velocity)
    {
        return velocity * (28.0 / (1.25 * (Math.PI * (72.0 / 25.4))));
    }

    public double getServoTicksFromDeg(double angle)
    {
        return 0.19 + (angle - 22.0) * (0.79 / 24.0); //TUNE
    }

    public double getDegFromServoTicks(double ticks)
    {
        return 22.0 + (ticks - 0.19) * (24.0 / 0.79);
    }

    public double getRadFromServoTicks(double ticks)
    {
        return Math.toRadians(getDegFromServoTicks(ticks));
    }

    public void zero()
    {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    public void killPower()
    {
        turretMotor.setPower(0);
        launcherMotorPrimary.setPower(0);
        launcherMotorPrimary.setPower(0);
        hoodAngleServo.setPwmDisable();
    }

    public double calculateAutoVelocity(double goalDist)
    {
        return Range.clip(((-0.0102645) * Math.pow(goalDist, 2)) + (7.49441 * goalDist) + 710, 0, 2500);
    }

    public double calculateAutoHoodAngle(double goalDist)
    {
        return Range.clip(((-0.00000688104) * Math.pow(goalDist, 2)) + (0.00816319 * goalDist) - 0.145203, 0, 1);
    }

    public double[] interpolatePower(double goalDist)
    {
        int n = Distances.length;

        if (goalDist <= Distances[0]) return new double[] {Velocities[0] + 10, HoodAngles[0]};
        if (goalDist >= Distances[n - 1]) return new double[] {Velocities[n - 1] + 10, HoodAngles[n - 1]};

        int low = 0;
        int high = n - 1;
        while (high - low > 1)
        {
            int mid = (low + high) >>> 1;
            if (goalDist >= Distances[mid]) low = mid;
            else high = mid;
        }

        double distance0 = Distances[low];
        double distance1 = Distances[high];
        double t = (goalDist - distance0) / (distance1 - distance0);

        double velocity0 = Velocities[low];
        double velocity1 = Velocities[high];
        double hood0 = HoodAngles[low];
        double hood1 = HoodAngles[high];

        double velocity = velocity0 + (velocity1 - velocity0) * t;
        double hoodAngle = hood0 + (hood1 - hood0) * t;

        return new double[] {velocity, hoodAngle};
    }

    public double getRealVelocity()
    {
        return -launcherMotorSecondary.getVelocity();
    }

    public double getExpectedVelocity()
    {
        return flywheelVelocity;
    }

    public double getHoodAngle()
    {
        return hoodAngle;
    }

    public double getDistToGoal()
    {
        return distToGoal;
    }

    public boolean flywheelAtExpectedSpeed()
    {
        return (-launcherMotorSecondary.getVelocity()) - flywheelVelocity >= -10;
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

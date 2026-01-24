package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constant.AllianceColor;
import org.firstinspires.ftc.teamcode.teleop.DriveMeet2;

public class TurretSubsystem
{
    private final DcMotorEx motor;
    private final GoBildaPinpointDriver odo;
    private final ServoImplEx launchAngle;
    private final DcMotorEx launcher;
    private final DcMotorEx launcher1;

    private double lineToGoal;
    private double angle = 0.18;
    private double vel = 1900;
    private double offset = 0;
    private double target;

    private final int TURRET_LEFT = -82;
    private final int TURRET_RIGHT = 45;


    public TurretSubsystem(HardwareMap hwMap)
    {
        motor = hwMap.get(DcMotorEx.class, "turret");
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        launchAngle = hwMap.get(ServoImplEx.class, "launchAngle");
        launcher = hwMap.get(DcMotorEx.class, "launcher");
        launcher1 = hwMap.get(DcMotorEx.class, "launcher1");

        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher1.setDirection(DcMotorSimple.Direction.REVERSE);

        launchAngle.setDirection(Servo.Direction.REVERSE);

        odo.setOffsets(-48, -182.5, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        odo.resetPosAndIMU();
    }


    public void update(boolean autoPower, double power, double hoodAngle, double offset, AllianceColor alliance, double robotX, double robotY, double robotHeading)
    {
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


        double goalX = alliance == AllianceColor.RED ? 144 : 140;
        double goalY = alliance == AllianceColor.RED ? 144 : 145;
        double distX = goalX - robotX;
        double distY = goalY - robotY;
        lineToGoal = Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));

        double deg = Math.toDegrees(Math.atan2(distY, distX));
        deg -= Math.toDegrees(robotHeading);
        deg += offset;
        deg = AngleUnit.normalizeDegrees(deg);
        deg = Range.clip(deg, TURRET_LEFT, TURRET_RIGHT);

//      double deg = offset;

        double target = deg;
        target = target * 5.6111111111;
        int fTarget = (int) Math.round(target * 384.5 / 360.0);
        this.target = fTarget;

        motor.setTargetPosition(fTarget);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);

    }

    public double autoPower(double goalDist) {
        return Range.clip((0.00785922 * Math.pow(goalDist, 3)) - (1.65427 * Math.pow(goalDist, 2)) + (117.05967 * goalDist) - 1464.50463, 0, 1800);
    }

    public double autoHood(double goalDist) {
        return Range.clip((5.64189e-7 * Math.pow(goalDist, 3)) - (0.000120574 * Math.pow(goalDist, 2)) + (0.00950386 * goalDist) + 0.00863624, 0, 0.5);
    }


    public void zero() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odo.resetPosAndIMU();
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
        private double velocity;

        public spinUpRaw(double vel) {
            this.velocity = vel;
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

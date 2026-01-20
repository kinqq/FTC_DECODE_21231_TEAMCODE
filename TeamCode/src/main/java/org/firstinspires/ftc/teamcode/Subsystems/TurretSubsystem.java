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

    private double angle = 0.18;
    private double vel = 1900;
    private double offset = 0;
    private double target;

    private final int TURRET_LEFT = -90;
    private final int TURRET_RIGHT = 90;


    public TurretSubsystem(HardwareMap hwMap)
    {
        motor = hwMap.get(DcMotorEx.class, "turret");
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        launchAngle = hwMap.get(ServoImplEx.class, "launchAngle");
        launcher = hwMap.get(DcMotorEx.class, "launcher");
        launcher1 = hwMap.get(DcMotorEx.class, "launcher1");

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


    public void update(double power, double hoodAngle, double offset, AllianceColor alliance, double robotX, double robotY, double robotHeading)
    {
        vel = 1800 * power;
        launchAngle.setPosition(hoodAngle);

        double goalX = alliance == AllianceColor.RED ? 135 : 10;
        double goalY = alliance == AllianceColor.RED ? 140 : 170;
        double distX = Math.abs(goalX - robotX);
        double distY = Math.abs(goalY - robotY);
        double lineToGoal = Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));

//        double deg = Math.toDegrees(Math.atan2(distY, distX));
//        deg -= Math.toDegrees(robotHeading);
//        deg += offset;
//        deg *= alliance == AllianceColor.BLUE ? -1 : 1;
//        deg = deg < TURRET_LEFT ? TURRET_RIGHT : deg;
//        deg = deg > TURRET_RIGHT ? TURRET_LEFT : deg;
        //deg = Range.clip(deg, TURRET_LEFT, TURRET_RIGHT);

        double deg = offset;

        double target = deg;
        target = target * 5.6111111111;
        int fTarget = (int) Math.round(target * 537.7 / 360.0);
        this.target = deg;

        motor.setTargetPosition(-fTarget);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);

    }

    public void zero() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odo.resetPosAndIMU();
    }

    public void setLaunchAngle(double angle) {
        this.angle = angle;
    }

    public class setLaunchAngle extends CommandBase
    {
        double g;

        public setLaunchAngle(double g) {
            this.g = g;
        }


        @Override
        public void initialize() {
            setLaunchAngle(g);
        }

        public boolean isFinished() {
            return true;
        }
    }

    public void launchAngleUp() {angle += 0.01;}
    public void launchAngleDown() {angle -= 0.01;}

    public double getLaunchAngle() {
        return angle;
    }

    public int getPos() {
        return motor.getCurrentPosition();
    }

    public double getTarget() {
        return target;
    }

    public void setLaunchVel(double vel) {
        this.vel = vel;
    }

    public void setOffset(double offset) {
        offset = 90 * offset;
        this.offset = offset;
    }

    public class setOffset extends CommandBase
    {
        double g;

        public setOffset(double g) {
            this.g = g;
        }


        @Override
        public void initialize() {
            setOffset(g);
        }

        public boolean isFinished() {
            return true;
        }
    }


    public void upOffset() {offset += 2;}
    public void downOffset() {offset -= 2;}

    public double getOffset() {
        return offset;
    }

    public double getVel() {
        return launcher1.getVelocity();
    }

    public double getExpectedVel() {
        return vel;
    }

    public class toggleSpin extends CommandBase {
        ElapsedTime timer = new ElapsedTime();

        public toggleSpin() {
        }

        @Override
        public void initialize() {
            if (launcher.getPower() == 1)
            {
                launcher.setPower(0);
                launcher1.setPower(0);
                launcher1.setVelocity(0);
            } else {
                launcher.setPower(1);
                launcher1.setPower(1);
                launcher1.setVelocity(vel);
            }

            timer.reset();
        }

        public boolean isFinished() {return motorToSpeed() || timer.seconds() > 5;}
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

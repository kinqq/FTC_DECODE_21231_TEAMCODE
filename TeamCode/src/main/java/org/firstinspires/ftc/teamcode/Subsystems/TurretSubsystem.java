package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
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
import org.firstinspires.ftc.teamcode.teleop.DriveMeet1;
import org.firstinspires.ftc.teamcode.teleop.DriveMeet2;

import dev.nextftc.core.commands.Command;

public class TurretSubsystem
{
    private DcMotorEx motor;
    private GoBildaPinpointDriver odo;
    private ServoImplEx launchAngle;
    private DcMotorEx launcher;

    private DcMotorEx encoder;

    private AnalogInput encoderAnalog;

    private double angle = 0.18;
    private double vel = 1900;
    private double offset = -5;

    public TurretSubsystem(HardwareMap hwMap)
    {
        motor = hwMap.get(DcMotorEx.class, "turret");
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        launchAngle = hwMap.get(ServoImplEx.class, "launchAngle");
        launcher = hwMap.get(DcMotorEx.class, "launcher");

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launchAngle.setDirection(Servo.Direction.REVERSE);
        launchAngle.scaleRange(0, .62);

        odo.setOffsets(-48, -182.5, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        odo.resetPosAndIMU();
    }

    public void update()
    {
        AllianceColor alliance = DriveMeet2.alliance;

        odo.update();

        launchAngle.setPosition(angle);

        double goalX = 900;
        if (alliance == AllianceColor.BLUE) goalX *= -1;
        double distX = -goalX - odo.getPosX(DistanceUnit.MM);
        double goalY = 1700;
        double distY = goalY - odo.getPosY(DistanceUnit.MM);
        double distFromGoal = Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));

        double deg = Math.toDegrees(Math.atan2(distX, distY)) + offset;
        deg -= odo.getHeading(AngleUnit.DEGREES);
        deg = Range.clip(deg, -190, 80);
        double target = Range.clip(deg - 10, -190, 80);
        ;//Range.clip(deg - 10, -235, 85);
        target = target * 5.6111111111;
        int fTarget = (int) Math.round(target * 537.7 / 360.0);


        motor.setTargetPosition(fTarget);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }

    public void zero() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setLaunchAngle(double angle) {
        this.angle = angle;
    }
    public void launchAngleUp() {angle += 0.05;}
    public void launchAngleDown() {angle -= 0.05;}

    public double getLaunchAngle() {
        return angle;
    }

    public void setLaunchVel(double vel) {
        this.vel = vel;
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }
    public void upOffset() {offset += 5;}
    public void downOffset() {offset -= 5;}

    public double getOffset() {
        return offset;
    }

    public class spinUp extends CommandBase {
        ElapsedTime timer = new ElapsedTime();

        public spinUp() {timer.reset();}

        @Override
        public void initialize() {
            launcher.setPower(1);
            launcher.setVelocity(vel);
        }

        public boolean isFinished() {return motorToSpeed() || timer.seconds() > 5;}
    }

    public class spinDown extends CommandBase {
        public spinDown() {}

        @Override
        public void initialize() {
            launcher.setPower(0);
        }

        @Override
        public boolean isFinished() {return true;}
    }

    public boolean motorToSpeed() {
        return launcher.getVelocity() <= vel + 250 && launcher.getVelocity() >= vel - 50;
    }
}

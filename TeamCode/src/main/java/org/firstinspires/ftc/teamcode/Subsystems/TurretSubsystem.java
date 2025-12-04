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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleop.DriveMeet1;

public class TurretSubsystem
{
    private DcMotorEx motor;
    private GoBildaPinpointDriver odo;
    private ServoImplEx launchAngle;

    private DcMotorEx encoder;

    private AnalogInput encoderAnalog;

    public TurretSubsystem(HardwareMap hwMap)
    {
        motor = hwMap.get(DcMotorEx.class, "turret");
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        launchAngle = hwMap.get(ServoImplEx.class, "launchAngle");

        launchAngle.setDirection(Servo.Direction.REVERSE);
        launchAngle.scaleRange(0.63, 0.97);

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
        odo.update();

        launchAngle.setPosition(0.25);

        double goalX = 900;
        double distX = -goalX - odo.getPosX(DistanceUnit.MM);
        double goalY = 1700;
        double distY = goalY - odo.getPosY(DistanceUnit.MM);
        double distFromGoal = Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));

        double deg = Math.toDegrees(Math.atan2(distX, distY));
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
}

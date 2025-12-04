package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.constant.ConstantsPIDF;
import org.firstinspires.ftc.teamcode.constant.ConstantsServo;
import org.firstinspires.ftc.teamcode.teleop.DriveMeet1;

public class MagazineSubsystem
{
    private final CRServoImplEx magazine;
    private final AnalogInput analog;
    private final DcMotorEx encoder;

    private int targetPos = 0;

    private boolean homed = false;


    private PIDController pid = new PIDController(Constants.SERVO_P, Constants.SERVO_I, Constants.SERVO_D);
    private PIDController pidHome = new PIDController(Constants.HOME_P, Constants.HOME_I, Constants.HOME_D);


    public MagazineSubsystem(HardwareMap hwMap)
    {
        magazine = hwMap.get(CRServoImplEx.class, "turntable");
        analog = hwMap.get(AnalogInput.class, "encoder");
        encoder = hwMap.get(DcMotorEx.class, "encoderDigital");
    }

    public boolean home()
    {
        pidHome.setPID(Constants.HOME_P, Constants.HOME_I, Constants.HOME_D);

        double voltage = analog.getVoltage();
        double angleDeg = (voltage / 3.3) * 360.0;

        double output = pidHome.calculate(angleDeg, 0);
        magazine.setPower(output);

        homed = angleDeg < 20;
        if (homed) encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return angleDeg < 20;
    }



    public void update()
    {
        pid.setPID(Constants.SERVO_P, Constants.SERVO_I, Constants.SERVO_D);

        int currentPosition = encoder.getCurrentPosition();

        double output = pid.calculate(currentPosition, targetPos);

        magazine.setPower(Range.clip(output, -1, 1));
    }

    public void setTarget(int target) {targetPos = target;}

    public int getTarget() {return targetPos;}

    public int getEncoder() {return encoder.getCurrentPosition();}
    public double getAnalog() {return (analog.getVoltage() / 3.3) * 360.0;}
    public double getPower() {return magazine.getPower();}
}
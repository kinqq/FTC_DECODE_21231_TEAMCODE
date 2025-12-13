package org.firstinspires.ftc.teamcode.test;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constant.ConstantsServo;

@TeleOp(name = "Magazine PIDF Tuner", group = "Tuning")
public class CRServoPIDFTuning extends OpMode {

    CRServoImplEx servo;
    public AnalogInput analog;
    public DcMotorEx encoder;

    @Override
    public void init() {
        servo = hardwareMap.get(CRServoImplEx.class, "turntable");

        analog = hardwareMap.get(AnalogInput.class, "encoder");
        encoder = hardwareMap.get(DcMotorEx.class, "encoderDigital");
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));


        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    int target = 0;
    @Override
    public void loop() {
        PIDController pid = new PIDController(ConstantsServo.kP, ConstantsServo.kI, ConstantsServo.kD);

        int currentWorldTicks = encoder.getCurrentPosition();
        int targetWorldTicks = target;

        double output = pid.calculate(currentWorldTicks, targetWorldTicks);

        int sign;
        if (output > 0)
        {
            sign = 1;
        }
        else
        {
            sign = -1;
            output *= -1;
        }
        output = Math.sqrt(output);
        output *= sign;
        servo.setPower(Range.clip(output, -1.0, 1.0));


        if (gamepad1.bWasPressed()) target = 0;
        if (gamepad1.aWasPressed()) target = 1333;
    }


}
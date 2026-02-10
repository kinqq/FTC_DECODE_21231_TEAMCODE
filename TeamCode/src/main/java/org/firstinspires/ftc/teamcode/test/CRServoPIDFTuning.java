package org.firstinspires.ftc.teamcode.test;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constant.ConstantsServo;
import org.firstinspires.ftc.teamcode.teleop.DriveMeet1;

@TeleOp(name = "Magazine PIDF Tuner", group = "Tuning")
@Disabled

public class CRServoPIDFTuning extends OpMode {

    CRServoImplEx servo;
    public CRServoImplEx servo1;
    public AnalogInput analog;
    public DcMotorEx encoder;

    @Override
    public void init() {
        servo = hardwareMap.get(CRServoImplEx.class, "turntable");
        servo1 = hardwareMap.get(CRServoImplEx.class, "turntable1");

        encoder = hardwareMap.get(DcMotorEx.class, "intake");
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));


        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        servo1.setDirection(DcMotorSimple.Direction.REVERSE);

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PanelsConfigurables.INSTANCE.refreshClass(DriveMeet1.class);
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }

    public int target = 0;

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
            //output *= -1;
        }
        //output = Math.sqrt(output);
       // output *= sign;

        //servo.setPower(1);
        //servo1.setPower(1);

        servo.setPower(Range.clip(output, -1.0, 1.0));
        servo1.setPower(Range.clip(output, -1, 1));

        if (gamepad1.bWasPressed()) target = 0;
        if (gamepad1.aWasPressed()) target = 1333;
        if (gamepad1.yWasPressed()) {target = 0; encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}

        telemetry.addData("Target", target);
        telemetry.addData("Encoder", encoder.getCurrentPosition());
        telemetry.addData("Power", servo.getPower());
        telemetry.addData("Power1", servo1.getPower());
        telemetry.addData("Output", output);
        telemetry.addLine();
        telemetry.addData("P", pid.getP());
        telemetry.addData("I", pid.getI());
        telemetry.addData("D", pid.getD());
    }


}
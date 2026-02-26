package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.constant.ConstantsServo;

@TeleOp(name = "TestServo", group = "Test")
@Configurable
public class TestServo extends OpMode {
    ServoImplEx servo;
    AnalogInput analogInput;

    @Override
    public void init() {
        servo = hardwareMap.get(ServoImplEx.class, "hammer");
        analogInput = hardwareMap.get(AnalogInput.class, "analog");
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }
    @Override
    public void loop() {
        servo.setPosition(ConstantsServo.targetDeg);

        telemetry.addData("targetDeg", ConstantsServo.targetDeg);
        telemetry.addData("analog", analogInput.getVoltage());
        telemetry.addData("angle", analogInput.getVoltage() * 109.0909090909);
        telemetry.update();
    }
}

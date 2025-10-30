package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TestServoAnalog", group = "Test")
@Configurable
public class TestServoAnalog extends OpMode {
    private ServoImplEx servo;          // 단일 CR 서보
    private final ElapsedTime timer = new ElapsedTime();
    private boolean increasing = true;
    private TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public static String servoName1 = "turntable";

    private AnalogInput analog;

    @Override
    public void init() {
        servo = (ServoImplEx) hardwareMap.get(Servo.class, servoName1);
        analog = hardwareMap.get(AnalogInput.class, "analog");

        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        servo.setPosition(0);

        telemetryM.addLine("Initialized!");
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        if (timer.milliseconds() > 1000) {
            if (increasing) {
                servo.setPosition(servo.getPosition() + 0.1);
                if (servo.getPosition() >= 1) increasing = false;
            }
            else {
                servo.setPosition(servo.getPosition() - 0.1);
                if (servo.getPosition() <= 0) increasing = true;
            }
            timer.reset();
        }

        // 아날로그 피드백 읽기 (0~3.3V) → 각도(예: 0~360° 가정; 필요시 보정)
        double voltage = analog.getVoltage();
        double angleDeg = (voltage / 3.3) * 360.0;

        telemetryM.addData("pos", servo.getPosition());
        telemetryM.addData("vol", voltage);
        telemetryM.addData("agl", angleDeg);
        telemetryM.update(telemetry);
    }
}

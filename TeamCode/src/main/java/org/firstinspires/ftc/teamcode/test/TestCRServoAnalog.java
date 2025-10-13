package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TestCRServoAnalog", group = "Test")
@Configurable
public class TestCRServoAnalog extends OpMode {
    private CRServo crServo;          // 단일 CR 서보
    public static String servoName1 = "turntable";

    private AnalogInput analog;       // 피드백(아날로그)
    public static double power = 0.0; // CR 서보 파워 [-1..1]
    public static double stepCoarse = 0.05; // A/B 증감 스텝
    public static double stepFine   = 0.01; // D-Pad Up/Down 미세 스텝

    @Override
    public void init() {
        crServo = hardwareMap.get(CRServo.class, servoName1);
        analog  = hardwareMap.get(AnalogInput.class, "analog");
        power   = 0.0; // 시작은 정지
    }

    @Override
    public void loop() {
        // 버튼 입력 (FrozenMilk 확장 gamepad 사용 가정: aWasPressed 등)
        if (gamepad1.aWasPressed())       power += stepCoarse;   // 시계방향 가속
        if (gamepad1.bWasPressed())       power -= stepCoarse;   // 반시계 가속
        if (gamepad1.dpadUpWasPressed())  power += stepFine;     // 미세 +
        if (gamepad1.dpadDownWasPressed())power -= stepFine;     // 미세 -

        // 안전 클램프
        power = Range.clip(power, -1.0, 1.0);

        // CR 서보 구동
        crServo.setPower(power);

        // 아날로그 피드백 읽기 (0~3.3V) → 각도(예: 0~360° 가정; 필요시 보정)
        double voltage = analog.getVoltage();
        double angleDeg = (voltage / 3.3) * 360.0;

        telemetry.addData("CR Power", "%.3f", power);
        telemetry.addData("Analog V", "%.3f V", voltage);
        telemetry.addData("Angle", "%.1f deg", angleDeg);
        telemetry.update();
    }

    @Override
    public void stop() {
        crServo.setPower(0);
    }
}

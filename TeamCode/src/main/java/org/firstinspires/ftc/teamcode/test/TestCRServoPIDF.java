package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.constant.ConstantsServo.kP;
import static org.firstinspires.ftc.teamcode.constant.ConstantsServo.kI;
import static org.firstinspires.ftc.teamcode.constant.ConstantsServo.kD;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.arcrobotics.ftclib.controller.PIDController;


@TeleOp(group = "Test")
@Configurable
public class TestCRServoPIDF extends OpMode {
    public static String servoName = "turntable";
    public static String encoderName = "encoderDigital";

    public static double TICKS_PER_REV = 16000.0;

    public static double stepLarge = 180;
    public static double stepMed   = 45.0;
    public static double stepFine  = 10.0;

    private CRServo crServo;
    private DcMotorEx encoder;
    private PIDController pid;

    public static double targetDeg = 0.0;
    private double lastCmd = 0.0;

    @Override
    public void init() {
        crServo = hardwareMap.get(CRServo.class, servoName);
        encoder = hardwareMap.get(DcMotorEx.class, encoderName);

        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        pid = new PIDController(kP, kI, kD);
        crServo.setPower(0);
    }

    @Override
    public void init_loop() {
        double measuredDeg = ticksToDeg(getEncoderTicks());
        telemetry.addData("Target", "%.2f deg", targetDeg);
        telemetry.addData("Measured", "%.2f deg", measuredDeg);
        telemetry.update();
    }

    @Override
    public void start() {
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        targetDeg = 0.0;
    }

    @Override
    public void loop() {
        pid.setPID(kP, kI, kD);

        if (gamepad1.rightBumperWasPressed()) targetDeg += stepLarge;
        if (gamepad1.leftBumperWasPressed())  targetDeg -= stepLarge;
        if (gamepad1.aWasPressed())           targetDeg += stepMed;
        if (gamepad1.bWasPressed())           targetDeg -= stepMed;
        if (gamepad1.dpadUpWasPressed())      targetDeg += stepFine;
        if (gamepad1.dpadDownWasPressed())    targetDeg -= stepFine;

        if (gamepad1.xWasPressed()) {
            encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            targetDeg = 0.0;
            pid.reset();
        }

        double targetTicks = targetDeg / 360 * TICKS_PER_REV;
        double pidOut = pid.calculate(getEncoderTicks(), targetTicks);

        lastCmd = Range.clip(pidOut, -1.0, 1.0);
        crServo.setPower(Math.copySign(Math.sqrt(Math.abs(lastCmd)), lastCmd));

        telemetry.addData("kP/kI/kD", "%.4f / %.4f / %.4f", kP, kI, kD);
        telemetry.addData("Target", "%.2f ticks", targetTicks);
        telemetry.addData("Measured", "%.2f ticks", getEncoderTicks());
        telemetry.addData("Err", "%.2f ticks", (targetTicks - getEncoderTicks()));
        telemetry.addData("Cmd", "%.3f", lastCmd);
        telemetry.update();
    }

    @Override
    public void stop() {
        crServo.setPower(0);
    }

    private double getEncoderTicks() {
        return encoder.getCurrentPosition();
    }

    private double ticksToDeg(double ticks) {
        return (ticks / TICKS_PER_REV) * 360.0;
    }
}

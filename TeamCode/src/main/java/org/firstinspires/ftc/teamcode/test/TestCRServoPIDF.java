package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.util.ConstantsServo.STATIC_COMP;
import static org.firstinspires.ftc.teamcode.util.ConstantsServo.kD_VEL;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.util.ConstantsServo;
import org.firstinspires.ftc.teamcode.util.ContinuousAbsoluteTracker;

import java.util.Locale;

@TeleOp(name = "TestServoPIDF", group = "Test")
@Configurable
public class TestCRServoPIDF extends OpMode {

    // --- Hardware Names ---
    public static String servoName  = "turret";
    public static String analogName = "analog";

    public static double wrapRangeDeg = 360.0;

    // --- Control Steps ---
    public static double stepLarge = 252.5;
    public static double stepMed   = 45.0;
    public static double stepFine  = 10.0;
    private static final double STATIC_ANGLE_THRESH = 1.0;

    // --- Prediction Settings ---
    private static final double LATENCY_SEC = 0.06;
    private static final double MAX_PREDICT_DEG = 10.0;

    // --- Objects ---
    private CRServo crServo;
    private AnalogInput analog;
    private ContinuousAbsoluteTracker tracker;
    private PIDController pid;

    // --- Runtime ---
    public static double targetDeg = 0.0;
    private double lastCmd = 0.0;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        crServo = hardwareMap.get(CRServo.class, servoName);
        analog  = hardwareMap.get(AnalogInput.class, analogName);

        tracker = new ContinuousAbsoluteTracker(analog, wrapRangeDeg);
        pid = new PIDController(ConstantsServo.kP, ConstantsServo.kI, ConstantsServo.kD);

        crServo.setPower(0);
        loopTimer.reset();
    }

    @Override
    public void init_loop() {
        tracker.updateAbsolute();
        telemetry.addData("Position", tracker.getEstimatedPosition());
        telemetry.addData("Target", tracker.getTotalAngleDeg());
        telemetry.update();
    }

    @Override
    public void start() {
        tracker.updateAbsolute();
        targetDeg = tracker.getTotalAngleDeg();
    }

    @Override
    public void loop() {
        // --- Update PID gains (live-tunable) ---
        pid.setPID(ConstantsServo.kP, ConstantsServo.kI, ConstantsServo.kD);

        // --- Gamepad Input (Setpoint Adjustments) ---
        if (gamepad1.rightBumperWasPressed()) targetDeg += stepLarge;
        if (gamepad1.leftBumperWasPressed())  targetDeg -= stepLarge;
        if (gamepad1.aWasPressed())           targetDeg += stepMed;
        if (gamepad1.bWasPressed())           targetDeg -= stepMed;
        if (gamepad1.dpadUpWasPressed())      targetDeg += stepFine;
        if (gamepad1.dpadDownWasPressed())    targetDeg -= stepFine;

        if (gamepad1.xWasPressed()) {
            tracker.rebaseAbsolute(0.0);
            targetDeg = 0.0;
            pid.reset();
        }

        // --- Sensor & Prediction ---
        tracker.updateAbsolute();
        double measuredDeg = tracker.getTotalAngleDeg();
        double velDps = tracker.getEstimatedVelocityDps();
        double errMeasured = targetDeg - measuredDeg;

        // latency compensation (simple linear prediction)
        double predictedDeg = measuredDeg + velDps * LATENCY_SEC;
        predictedDeg = measuredDeg + Range.clip(predictedDeg - measuredDeg, -MAX_PREDICT_DEG, MAX_PREDICT_DEG);

        // --- PID Calculation ---
        double pidOut = pid.calculate(predictedDeg, targetDeg);

        // velocity-based damping (acts like "soft brake")
        double velDamping = -kD_VEL * velDps;
        double out = pidOut + velDamping;
        double rawOut = out;

        // static friction compensation
        if (Math.abs(out) > 0 && Math.abs(out) < STATIC_COMP && Math.abs(errMeasured) > STATIC_ANGLE_THRESH) {
            out = Math.signum(out) * STATIC_COMP;
        }

        // --- Output Power ---
        lastCmd = Range.clip(out, -1.0, 1.0);
        crServo.setPower(lastCmd);

        // --- Telemetry ---
        telemetry.addData("kP/kI/kD", String.format(Locale.US, "%.4f / %.4f / %.4f",
            ConstantsServo.kP, ConstantsServo.kI, ConstantsServo.kD));
        telemetry.addData("Target", "%.2f deg", targetDeg);
        telemetry.addData("Predicted", "%.2f deg", predictedDeg);
        telemetry.addData("Measured", "%.2f deg", measuredDeg);
        telemetry.addData("Err", "%.2f deg", (targetDeg - predictedDeg));
        telemetry.addData("Vel", "%.2f dps", velDps);
        telemetry.addData("PIDout", "%.4f", pidOut);
        telemetry.addData("VelDamp", "%.4f", velDamping);
        telemetry.addData("RawOut", "%.4f", rawOut);
        telemetry.addData("Cmd", "%.4f", lastCmd);
        telemetry.addData("Analog V", "%.3f V", tracker.getVoltage());
        telemetry.addData("Est Pos (0..1)", "%.3f", tracker.getEstimatedPosition());
        telemetry.update();

        loopTimer.reset();
    }

    @Override
    public void stop() {
        crServo.setPower(0);
    }
}

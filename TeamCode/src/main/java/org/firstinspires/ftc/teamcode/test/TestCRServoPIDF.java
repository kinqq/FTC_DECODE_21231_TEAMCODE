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

// FTCLib PID (verify your FTCLib version's class & signature)
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.util.ConstantsServo;

import java.util.Locale;

@TeleOp(name = "TestServoPIDF", group = "Test")
@Configurable
public class TestCRServoPIDF extends OpMode {
    public static String servoName  = "turntable";
    public static String analogName = "analog";

    private static final double CAL_A = 3.218;
    private static final double CAL_B = 0.008;

    public static double wrapRangeDeg = 360.0;

    public static double stepLarge = 180.0;
    public static double stepMed   = 45.0;
    public static double stepFine  = 15.0;
    private static final double STATIC_ANGLE_THRESH = 1.0;

    private static final double BRAKE_ERR_THRESH_DEG = 3.0;
    private static final double BRAKE_VEL_THRESH_DPS = 25.0;
    private static final double BRAKE_POWER = -0.07;
    private static final double MAX_BRAKE_TIME = 0.04;
    private static final double BRAKE_COOLDOWN = 0.12;
    private boolean brakeActive = false;
    private double brakeStartTime = -10.0;

    // latency for prediction (tune)
    private static final double LATENCY_SEC = 0.06;
    private static final double MAX_PREDICT_DEG = 10.0;

    private CRServo crServo;
    private AnalogInput analog;
    private ContinuousAbsoluteTracker tracker;
    private PIDController pid;

    public static double targetDeg = 0.0;
    private double lastCmd = 0.0;

    // timers: loopTimer -> dt; runtimeTimer -> absolute time (cooldown, braking)
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime runtimeTimer = new ElapsedTime();
    private double lastTargetDeg = 0.0;

    @Override
    public void init() {
        crServo = hardwareMap.get(CRServo.class, servoName);
        analog  = hardwareMap.get(AnalogInput.class, analogName);

        tracker = new ContinuousAbsoluteTracker(analog, wrapRangeDeg);

        // use your ConstantsServo gains (or replace with hard-coded initial tuning)
        pid = new PIDController(ConstantsServo.kP, ConstantsServo.kI, ConstantsServo.kD);

        tracker.updateAbsolute();
        targetDeg = tracker.getTotalAngleDeg();

        loopTimer.reset();
        runtimeTimer.reset();
        brakeStartTime = -10.0;
        lastTargetDeg = targetDeg;
    }

    @Override
    public void loop() {
        // reapply gains so they can be changed live (dashboard)
        pid.setPID(ConstantsServo.kP, ConstantsServo.kI, ConstantsServo.kD);

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

        // --- sensor update & prediction ---
        tracker.updateAbsolute();
        double measuredDeg = tracker.getTotalAngleDeg();
        double velDps = tracker.getEstimatedVelocityDps();
        double errMeasured = targetDeg - measuredDeg;

        double predictedDeg = measuredDeg + velDps * LATENCY_SEC;
        predictedDeg = measuredDeg + Range.clip(predictedDeg - measuredDeg, -MAX_PREDICT_DEG, MAX_PREDICT_DEG);

        // NOTE: check your FTCLib PIDController.calculate signature.
        // If it expects (measurement, setpoint) -> use pid.calculate(predictedDeg, targetDeg).
        // If it expects (setpoint, measurement) -> use pid.calculate(targetDeg, predictedDeg).
        double pidOut = pid.calculate(predictedDeg, targetDeg);

        // vel-based damping
        double velDamping = -kD_VEL * velDps;
        double out = pidOut + velDamping;
        double rawOut = out;

        // static friction compensation
        if (Math.abs(out) > 0 && Math.abs(out) < STATIC_COMP && Math.abs(errMeasured) > STATIC_ANGLE_THRESH) {
            out = Math.signum(out) * STATIC_COMP;
        }

        // --- braking pulse logic using runtimeTimer (absolute time) ---
        double now = runtimeTimer.seconds();
        if (!brakeActive) {
            if (Math.abs(errMeasured) < BRAKE_ERR_THRESH_DEG && Math.abs(velDps) > BRAKE_VEL_THRESH_DPS && (now - brakeStartTime) > BRAKE_COOLDOWN) {
                brakeActive = true;
                brakeStartTime = now;
            }
        }

        if (brakeActive) {
            if ((now - brakeStartTime) < MAX_BRAKE_TIME) {
                lastCmd = Range.clip(BRAKE_POWER, -1.0, 1.0) * Math.signum(out);
                crServo.setPower(lastCmd);
                telemetry.addData("Brake", "ACTIVE");
            } else {
                brakeActive = false;
            }
        } else {
            lastCmd = Range.clip(out, -1.0, 1.0);
            crServo.setPower(lastCmd);
        }

        // telemetry - show constants explicitly (safer than getCoefficients())
        telemetry.addData("kP/kI/kD", String.format(Locale.US, "%.4f / %.4f / %.4f", ConstantsServo.kP, ConstantsServo.kI, ConstantsServo.kD));
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

        // update bookkeeping
        lastTargetDeg = targetDeg;
        loopTimer.reset(); // reset loop timer at end (so dt is valid next iteration)
    }

    @Override
    public void stop() {
        crServo.setPower(0);
    }

    // --- Minimal continuous absolute tracker using linear calibration ---
    public static class ContinuousAbsoluteTracker {
        private final AnalogInput analog;
        private final double wrapDeg;

        private double lastWrappedDeg = Double.NaN;
        private int turns = 0;
        private double totalDeg = 0.0;

        private double lastTotalDeg = 0.0;
        private double velDps = 0.0;
        private final ElapsedTime t = new ElapsedTime();

        public ContinuousAbsoluteTracker(AnalogInput analog, double wrapDeg) {
            this.analog = analog;
            this.wrapDeg = wrapDeg;
            t.reset();
        }

        private double mapVtoDeg(double v) {
            double pos = (v - CAL_B) / CAL_A;
            pos = Range.clip(pos, 0.0, 1.0);
            return pos * wrapDeg;
        }

        public void updateAbsolute() {
            double v = analog.getVoltage();
            double wrapped = mapVtoDeg(v);

            if (Double.isNaN(lastWrappedDeg)) {
                lastWrappedDeg = wrapped;
                totalDeg = wrapped;
                lastTotalDeg = totalDeg;
                t.reset();
                return;
            }

            double delta = wrapped - lastWrappedDeg;
            double thresh = wrapDeg * 0.5;
            if (delta > thresh) turns -= 1;
            if (delta < -thresh) turns += 1;

            totalDeg = turns * wrapDeg + wrapped;

            double dt = Math.max(1e-3, t.seconds());
            velDps = (totalDeg - lastTotalDeg) / dt;
            lastTotalDeg = totalDeg;
            t.reset();

            lastWrappedDeg = wrapped;
        }

        public void rebaseAbsolute(double newBaseDeg) {
            updateAbsolute();
            double current = getTotalAngleDeg();
            double shift = newBaseDeg - current;
            totalDeg = current + shift;
            lastTotalDeg = totalDeg;
            turns = (int)Math.floor(totalDeg / wrapDeg);
        }

        public double getTotalAngleDeg()       { return totalDeg; }
        public double getEstimatedVelocityDps(){ return velDps; }
        public double getVoltage()             { return analog.getVoltage(); }
        public double getEstimatedPosition() {
            double v = getVoltage();
            double pos = (v - CAL_B) / CAL_A;
            return Range.clip(pos, 0.0, 1.0);
        }
    }
}

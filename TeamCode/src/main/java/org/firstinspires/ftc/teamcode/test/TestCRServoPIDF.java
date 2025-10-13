package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.util.ConstantsServo.STATIC_COMP;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// FTCLib PIDF
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.ConstantsServo;

import java.util.Arrays;

@TeleOp(name = "TestServoPIDF", group = "Test")
@Configurable
public class TestCRServoPIDF extends OpMode {
    // --- hardware names (changeable from DS) ---
    public static String servoName  = "turntable";
    public static String analogName = "analog";

    // --- calibration (from measured data) ---
    // voltage = CAL_A * pos + CAL_B
    private static final double CAL_A = 3.218;
    private static final double CAL_B = 0.008;

    // --- angular mapping ---
    public static double wrapRangeDeg = 360.0;  // full wrap range in degrees

    // --- simple jog / step settings ---
    public static double stepLarge = 180.0;
    public static double stepMed   = 45.0;
    public static double stepFine  = 15.0;
    private static final double JOG_DEADBAND = 0.05;   // measured deadband (min power that overcomes stiction)
    private static final double STATIC_ANGLE_THRESH = 1.0; // deg: 이 안의 오차면 static 보상 안함


    // --- internal state ---
    private CRServo crServo;
    private AnalogInput analog;
    private ContinuousAbsoluteTracker tracker;
    private PIDFController pidf;

    public static double targetDeg = 0.0;
    private double lastCmd = 0.0;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        crServo = hardwareMap.get(CRServo.class, servoName);
        analog  = hardwareMap.get(AnalogInput.class, analogName);

        // tracker uses the calibrated linear mapping to produce wrapped angle (0..wrapRangeDeg)
        tracker = new ContinuousAbsoluteTracker(analog, wrapRangeDeg);

        // create PIDF controller (you can replace Constants with explicit gains if desired)
        pidf = new PIDFController(ConstantsServo.kP, ConstantsServo.kI, ConstantsServo.kD, ConstantsServo.kF);

        // seed tracker and set initial target to current angle to avoid setpoint jumps
        tracker.updateAbsolute();
        targetDeg = tracker.getTotalAngleDeg();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = Math.max(1e-3, loopTimer.seconds());
        // ensure PIDF gains are reapplied each loop (allows live-tuning via Constants or dashboard)
        pidf.setPIDF(ConstantsServo.kP, ConstantsServo.kI, ConstantsServo.kD, ConstantsServo.kF);
        loopTimer.reset();

        // --- target adjustments via buttons ---
        if (gamepad1.rightBumperWasPressed()) targetDeg += stepLarge;
        if (gamepad1.leftBumperWasPressed())  targetDeg -= stepLarge;
        if (gamepad1.aWasPressed())           targetDeg += stepMed;
        if (gamepad1.bWasPressed())           targetDeg -= stepMed;
        if (gamepad1.dpadUpWasPressed())      targetDeg += stepFine;
        if (gamepad1.dpadDownWasPressed())    targetDeg -= stepFine;

        if (gamepad1.xWasPressed()) {
            // rebase current position to 0°
            tracker.rebaseAbsolute(0.0);
            targetDeg = 0.0;
            pidf.reset();
        }

        // --- sensor update ---
        tracker.updateAbsolute();
        double currentDeg = tracker.getTotalAngleDeg();
        double velDps = tracker.getEstimatedVelocityDps();

        double out = pidf.calculate(currentDeg, targetDeg);

        // Optional: small-angle deadband to avoid hunting near setpoint
        double err = targetDeg - currentDeg;
        if (Math.abs(err) < 0.2) { // 0.2 deg deadband (tune if necessary)
            out = 0.0;
        }

        // Static friction (minimum command) compensation
        if (Math.abs(out) > 0 && Math.abs(out) < STATIC_COMP && Math.abs(err) > STATIC_ANGLE_THRESH) {
            // apply minimum command in the direction of the error
            out = Math.signum(out) * STATIC_COMP;
        }

        // Clip and send
        lastCmd = Range.clip(out, -1.0, 1.0);
        crServo.setPower(lastCmd);

        // --- telemetry ---
        telemetry.addLine("=== CR + PIDF (calibrated) ===");
        telemetry.addData("PIDF", Arrays.toString(pidf.getCoefficients()));
        telemetry.addData("Target", "%.2f deg", targetDeg);
        telemetry.addData("Pos", "%.2f deg", currentDeg);
        telemetry.addData("Err", "%.2f deg", (targetDeg - currentDeg));
        telemetry.addData("Vel", "%.2f dps", velDps);
        telemetry.addData("Cmd", "%.5f", lastCmd);
        telemetry.addData("Analog V", "%.3f V", tracker.getVoltage());
        telemetry.addData("Est Pos (0..1)", "%.3f", tracker.getEstimatedPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        crServo.setPower(0);
    }

    // --- Minimal continuous absolute tracker using the linear calibration ---
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

        /** Convert measured voltage to normalized position [0..1] using the calibration,
         * then to wrapped degrees [0..wrapDeg]. */
        private double mapVtoDeg(double v) {
            // inverse calibration: pos = (v - CAL_B) / CAL_A
            double pos = (v - CAL_B) / CAL_A;
            pos = Range.clip(pos, 0.0, 1.0);
            return pos * wrapDeg;
        }

        public void updateAbsolute() {
            double v = analog.getVoltage();
            double wrapped = mapVtoDeg(v); // 0..wrapDeg

            if (Double.isNaN(lastWrappedDeg)) {
                // first frame: seed values
                lastWrappedDeg = wrapped;
                totalDeg = wrapped;
                lastTotalDeg = totalDeg;
                t.reset();
                return;
            }

            double delta = wrapped - lastWrappedDeg;

            // threshold for detecting wrap-around (simple heuristic: >50% wrap)
            double thresh = wrapDeg * 0.5;
            if (delta >  thresh) turns -= 1;
            if (delta < -thresh) turns += 1;

            totalDeg = turns * wrapDeg + wrapped;

            double dt = Math.max(1e-3, t.seconds());
            velDps = (totalDeg - lastTotalDeg) / dt;
            lastTotalDeg = totalDeg;
            t.reset();

            lastWrappedDeg = wrapped;
        }

        /** Rebase the absolute total angle so that current becomes newBaseDeg. */
        public void rebaseAbsolute(double newBaseDeg) {
            updateAbsolute();
            double current = getTotalAngleDeg();
            double shift = newBaseDeg - current;
            totalDeg = current + shift;
            lastTotalDeg = totalDeg;
            turns = (int)Math.floor(totalDeg / wrapDeg);
        }

        // getters
        public double getTotalAngleDeg()       { return totalDeg; }
        public double getEstimatedVelocityDps(){ return velDps; }
        public double getVoltage()             { return analog.getVoltage(); }
        /** Return estimated position in [0..1] using the calibration inverse. */
        public double getEstimatedPosition() {
            double v = getVoltage();
            double pos = (v - CAL_B) / CAL_A;
            return Range.clip(pos, 0.0, 1.0);
        }
    }
}

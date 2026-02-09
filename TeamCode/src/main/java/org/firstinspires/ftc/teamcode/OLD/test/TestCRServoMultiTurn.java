package org.firstinspires.ftc.teamcode.OLD.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TestServoMT", group = "Test")
@Configurable
@Disabled

public class TestCRServoMultiTurn extends OpMode {
    // ───── HW ───────────────────────────────────────────────────────────────────
    private CRServo crServo;
    private AnalogInput analog;

    // ───── Names / tunables ────────────────────────────────────────────────────
    public static String servoName = "turret";
    public static String analogName = "analog";

    // CR servo power step
    public static double stepCoarse = 0.05;
    public static double stepFine   = 0.01;

    // Tracker calibration defaults (현장 보정 권장)
    public static double vMin = 0.01;      // 최소 전압(실측으로 조정)
    public static double vMax = 3.29;      // 최대 전압(실측으로 조정)
    public static double wrapRangeDeg = 360.0;  // Axon 피드백이 0~360° 매핑된다고 가정(설정에 따라 조정)
    public static double hysteresisDeg = 20.0;  // 랩 감지 여유
    public static double filterAlpha   = 0.1;  // 0~1 (값↑ → 반응↑, 노이즈↑)

    private double power = 0.0;
    private boolean filterOn = true;

    private ContinuousServoTracker tracker;

    @Override
    public void init() {
        crServo = hardwareMap.get(CRServo.class, servoName);
        analog  = hardwareMap.get(AnalogInput.class, analogName);

        tracker = new ContinuousServoTracker(analog)
            .setCalibration(vMin, vMax, wrapRangeDeg)
            .setHysteresis(hysteresisDeg)
            .setFilterAlpha(filterAlpha)
            .enableFilter(filterOn);

        tracker.reset(0.0); // 시작 기준 0°
    }

    @Override
    public void loop() {
        // ── CRServo power control ───────────────────────────────────────────────
        if (gamepad1.aWasPressed())        power += stepCoarse;
        if (gamepad1.bWasPressed())        power -= stepCoarse;
        if (gamepad1.dpadUpWasPressed())   power += stepFine;
        if (gamepad1.dpadDownWasPressed()) power -= stepFine;
        power = Range.clip(power, -1.0, 1.0);
        crServo.setPower(power);

        // ── Utilities: zero / filter toggle ────────────────────────────────────
        if (gamepad1.xWasPressed()) {
            // 현재 각도를 0° 기준으로 리셋 (누적 기준 재설정)
            tracker.reset(0.0);
        }
        if (gamepad1.yWasPressed()) {
            filterOn = !filterOn;
            tracker.enableFilter(filterOn);
        }

        // ── Update tracker ─────────────────────────────────────────────────────
        tracker.update();

        // ── Telemetry ──────────────────────────────────────────────────────────
        telemetry.addLine("=== CR Servo + Multi-Turn Tracking ===");
        telemetry.addData("Power", "%.3f", power);
        telemetry.addData("Analog V", "%.3f V", tracker.getVoltage());
        telemetry.addData("Angle (wrapped)", "%.1f deg", tracker.getWrappedAngleDeg());
        telemetry.addData("Turns", tracker.getTurns());
        telemetry.addData("Total Angle", "%.1f deg", tracker.getTotalAngleDeg());
        telemetry.addData("Filter", filterOn ? "ON" : "OFF");
        telemetry.addData("Map", "vMin=%.2f vMax=%.2f wrap=%.0f°", vMin, vMax, wrapRangeDeg);
        telemetry.addData("Hint", "X=zero, Y=filter toggle, A/B/DPad=power");
        telemetry.update();
    }

    @Override
    public void stop() {
        crServo.setPower(0);
    }

    // ───────────────────────────────────────────────────────────────────────────
    //  ContinuousServoTracker (same-file helper class)
    // ───────────────────────────────────────────────────────────────────────────
    public static class ContinuousServoTracker {
        private final AnalogInput analog;

        private double vMin = 0.0;
        private double vMax = 3.3;
        private double wrapRangeDeg = 360.0;

        private double alpha = 0.4;
        private boolean useFilter = true;

        private double lastAngleDegRaw = Double.NaN; // 0..wrap
        private double lastAngleDeg    = Double.NaN; // filtered 0..wrap
        private int    turns = 0;
        private double totalAngleDeg = 0.0;
        private double hysteresisDeg = 20.0;

        @SuppressWarnings("unused")
        private final ElapsedTime timer = new ElapsedTime();

        public ContinuousServoTracker(AnalogInput analog) {
            this.analog = analog;
        }

        public ContinuousServoTracker setCalibration(double vMin, double vMax, double wrapRangeDeg) {
            this.vMin = vMin;
            this.vMax = vMax;
            this.wrapRangeDeg = wrapRangeDeg;
            return this;
        }

        public ContinuousServoTracker setHysteresis(double hysteresisDeg) {
            this.hysteresisDeg = Math.max(5.0, hysteresisDeg);
            return this;
        }

        public ContinuousServoTracker setFilterAlpha(double alpha) {
            this.alpha = Math.min(1.0, Math.max(0.0, alpha));
            return this;
        }

        public ContinuousServoTracker enableFilter(boolean enable) {
            this.useFilter = enable;
            return this;
        }

        public void reset(double totalAngleStartDeg) {
            lastAngleDegRaw = Double.NaN;
            lastAngleDeg    = Double.NaN;
            totalAngleDeg   = totalAngleStartDeg;
            turns           = (int)Math.floor(totalAngleStartDeg / wrapRangeDeg);
        }

        public void update() {
            double v = analog.getVoltage();
            double angleDegRaw = mapVoltageToAngle(v);

            double angleDeg = angleDegRaw;
            if (useFilter) {
                if (!Double.isNaN(lastAngleDeg)) {
                    angleDeg = lastAngleDeg + alpha * (angleDegRaw - lastAngleDeg);
                }
            }
            if (Double.isNaN(lastAngleDeg)) {
                lastAngleDeg = angleDeg;
                lastAngleDegRaw = angleDegRaw;
                return;
            }

            double delta = angleDeg - lastAngleDeg;

            // 랩 판정 경계: 반바퀴보다 여유를 둔 0.6*wrap + 히스테리시스
            double wrapThreshold = (wrapRangeDeg * 0.4) + hysteresisDeg;

            if (delta >  wrapThreshold) {
                // 350° -> 10° 같은 역방향 점프(반시계 랩)
                turns -= 1;
            } else if (delta < -wrapThreshold) {
                // 10° -> 350° 같은 순방향 점프(시계 랩)
                turns += 1;
            }

            totalAngleDeg = turns * wrapRangeDeg + angleDeg;

            lastAngleDeg = angleDeg;
            lastAngleDegRaw = angleDegRaw;
        }

        public double getTotalAngleDeg() { return totalAngleDeg; }
        public double getWrappedAngleDeg() { return lastAngleDeg; }
        public int getTurns() { return turns; }
        public double getVoltage() { return analog.getVoltage(); }

        private double mapVoltageToAngle(double v) {
            double vv = Math.max(vMin, Math.min(vMax, v));
            double t  = (vv - vMin) / Math.max(1e-6, (vMax - vMin)); // 0..1
            return t * wrapRangeDeg;
        }
    }
}

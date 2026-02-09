package org.firstinspires.ftc.teamcode.OLD.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ContinuousAbsoluteTracker {
    private final AnalogInput analog;
    private final double wrapDeg;

    private double lastWrappedDeg = Double.NaN;
    private int turns = 0;
    private double totalDeg = 0.0;

    private double offsetDeg = 0.0;      // <- zero/offset accumulator

    private double lastTotalDeg = 0.0;
    private double velDps = 0.0;
    private final ElapsedTime t = new ElapsedTime();

    private static final double CAL_A = 3.218;
    private static final double CAL_B = 0.008;

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
            totalDeg = wrapped + offsetDeg;  // include offset
            lastTotalDeg = totalDeg;
            t.reset();
            return;
        }

        double delta = wrapped - lastWrappedDeg;
        double thresh = wrapDeg * 0.5;
        if (delta > thresh) turns -= 1;
        if (delta < -thresh) turns += 1;

        totalDeg = turns * wrapDeg + wrapped + offsetDeg;  // include offset

        double dt = Math.max(1e-3, t.seconds());
        velDps = (totalDeg - lastTotalDeg) / dt;
        lastTotalDeg = totalDeg;
        t.reset();

        lastWrappedDeg = wrapped;
    }

    /** Rebase current absolute angle to newBaseDeg (e.test., zeroHere -> 0). */
    public void rebaseAbsolute(double newBaseDeg) {
        updateAbsolute();
        double current = getTotalAngleDeg();
        offsetDeg += (newBaseDeg - current);  // shift reference persistently
        lastTotalDeg = newBaseDeg;
        t.reset();
    }

    public double getTotalAngleDeg()        { return totalDeg; }
    public double getEstimatedVelocityDps() { return velDps; }
    public double getVoltage()              { return analog.getVoltage(); }
    public double getEstimatedPosition() {
        double v = getVoltage();
        double pos = (v - CAL_B) / CAL_A;
        return Range.clip(pos, 0.0, 1.0);
    }
}

package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.EnumMap;

import static org.firstinspires.ftc.teamcode.util.Constants.*;

import org.firstinspires.ftc.teamcode.util.DetectedColor;
import org.firstinspires.ftc.teamcode.util.Slot;

/**
 * This is basically just a turntable class without color sensing.
 * It uses handles user-input to get colors of artifacts.
 */
public class Magazine {
    private static final String TURN_SERVO_NAME   = "turntable";
    private static final String HAMMER_SERVO_NAME = "hammer";
    private static final double SLOT1_SENSE_POS = 0.32;
    private static final double SLOT2_SENSE_POS = 0.67;
    private static final double SLOT3_SENSE_POS = 1.00;
    private static final double SENSE_DWELL_MS  = 400;

    private static final double MS_PER_TRAVEL = 800;
    private static final double SETTLE_MIN_MS = 150;
    private static final double SETTLE_MAX_MS = 900;


    private static final int SAMPLE_TARGET = 12;

    private final ServoImplEx table;
    private final ServoImplEx hammer;

    private final EnumMap<Slot, DetectedColor> lastColor = new EnumMap<>(Slot.class);

    private Slot slot = Slot.FIRST;

    private enum State {
        IDLE,
        // Launch
        MOVING, FIRING, RETURNING,
        // nextSlot sensing
        NEXT_MOVING_TO_SENSE, NEXT_MOVING_TO_HOLD
    }
    private State state = State.IDLE;

    private final ElapsedTime timer = new ElapsedTime();
    private double pendingShootPos;
    private Slot   pendingNextSlot;

    private Slot   nextSlotTarget;

    private double currentSettleMs = 0;

    private int sampleCount = 0;
    private float sumH = 0f, sumS = 0f, sumV = 0f;

    public Magazine(HardwareMap hardwareMap) {
        this.table  = hardwareMap.get(ServoImplEx.class, TURN_SERVO_NAME);
        this.hammer = hardwareMap.get(ServoImplEx.class, HAMMER_SERVO_NAME);

        PwmRange range = new PwmRange(500, 2500);
        this.table.setPwmRange(range);
        this.hammer.setPwmRange(range);

        lastColor.put(Slot.FIRST,  DetectedColor.UNKNOWN);
        lastColor.put(Slot.SECOND, DetectedColor.UNKNOWN);
        lastColor.put(Slot.THIRD,  DetectedColor.UNKNOWN);
    }

    public void init() {
        this.hammer.setPosition(HAMMER_REST);
        applyHold(slot);
    }



    public void setSlot(Slot s) {
        if (s == null || isBusy()) return;
        slot = s;
        applyHold(slot);
    }

    public void nextSlot() {
        if (isBusy()) return;
        nextSlotTarget = next(slot);

        double target = holdPosOf(nextSlotTarget);
        setRaw(target);
        currentSettleMs = computeSettleMs(target);
        timer.reset();
        state = State.NEXT_MOVING_TO_HOLD;
    }

    public void prevSlot() {
        if (isBusy()) return;
        slot = prev(slot);
        applyHold(slot);
    }


    public void autoLaunch() {
        startLaunch(Slot.SECOND);
        while (state != State.IDLE);
        startLaunch(Slot.THIRD);
        while (state != State.IDLE);
        startLaunch(Slot.FIRST);
    }

    public void startLaunch(Slot s) {
        if (state != State.IDLE) return;

        pendingShootPos = shootingPosOf(s);
        pendingNextSlot = next(s);

        setRaw(pendingShootPos);
        timer.reset();
        state = State.MOVING;
        lastColor.put(s, DetectedColor.UNKNOWN);
    }

    public boolean tryStartLaunch(Slot s) {
        if (state != State.IDLE) return false;  // launch only from IDLE
        startLaunch(s);                          // your existing method
        // If startLaunch succeeded, state will now be MOVING
        return state != State.IDLE;
    }


    public void startLaunch(DetectedColor color) {
        if (isBusy()) return;

        java.util.ArrayList<Slot> candidates = new java.util.ArrayList<>(3);
        if (getLastColor(Slot.FIRST)  == color) candidates.add(Slot.FIRST);
        if (getLastColor(Slot.SECOND) == color) candidates.add(Slot.SECOND);
        if (getLastColor(Slot.THIRD)  == color) candidates.add(Slot.THIRD);
        if (candidates.isEmpty()) return;

        double current = getTablePos();
        Slot best = candidates.get(0);
        double bestDist = Math.abs(shootingPosOf(best) - current);

        for (int i = 1; i < candidates.size(); i++) {
            Slot ss = candidates.get(i);
            double dist = Math.abs(shootingPosOf(ss) - current);
            if (dist < bestDist) {
                bestDist = dist;
                best = ss;
            }
        }
        startLaunch(best);
    }


    public void update() {


        switch (state) {
            case IDLE:
                break;

            /* ---------- Launch ---------- */
            case MOVING:
                //TODO: add getVelocity for launch motor and
                if (timer.milliseconds() >= SETTLE_MS) { // Constants의 SETTLE_MS 사용(발사용)
                    hammer.setPosition(HAMMER_FIRE);     // Constants의 HAMMER_FIRE
                    timer.reset();
                    state = State.FIRING;
                }
                break;

            case FIRING:
                if (timer.milliseconds() >= FIRE_MS) {   // Constants의 FIRE_MS
                    hammer.setPosition(HAMMER_REST);     // Constants의 HAMMER_REST
                    slot = pendingNextSlot;
                    applyHold(slot);
                    timer.reset();
                    state = State.RETURNING;
                }
                break;

            case RETURNING:
                if (timer.milliseconds() >= RESET_MS) {  // Constants의 RESET_MS
                    state = State.IDLE;
                }
                break;

            /* ---------- nextSlot  ---------- */
            case NEXT_MOVING_TO_HOLD:
                if (timer.milliseconds() >= currentSettleMs) {
                    slot = nextSlotTarget;
                    state = State.IDLE;
                }
                break;
        }
    }


    public DetectedColor getLastColor(Slot s) {
        DetectedColor dc = lastColor.get(s);
        return dc == null ? DetectedColor.UNKNOWN : dc;
    }

    public void setRaw(double pos) {
        table.setPosition(Range.clip(pos, 0.0, 1.0));
    }

    public void setColors(DetectedColor slot1, DetectedColor slot2, DetectedColor slot3) {
        lastColor.put(Slot.FIRST, slot1);
        lastColor.put(Slot.SECOND, slot2);
        lastColor.put(Slot.THIRD, slot3);
    }

    public Slot getSlot()      { return slot; }
    public double getTablePos()  { return table.getPosition(); }
    public boolean isBusy()      { return state != State.IDLE; }



    private static Slot next(Slot s) {
        switch (s) {
            case FIRST:  return Slot.SECOND;
            case SECOND: return Slot.THIRD;
            case THIRD:  return Slot.FIRST;
            default:     return Slot.FIRST;
        }
    }

    private static Slot prev(Slot s) {
        switch (s) {
            case FIRST:  return Slot.THIRD;
            case SECOND: return Slot.FIRST;
            case THIRD:  return Slot.SECOND;
            default:     return Slot.FIRST;
        }
    }

    private static double holdPosOf(Slot s) {
        switch (s) {
            case FIRST:  return SLOT1_HOLD;
            case SECOND: return SLOT2_HOLD;
            case THIRD:  return SLOT3_HOLD;
            default:     return SLOT1_HOLD;
        }
    }

    private static double shootingPosOf(Slot s) {
        switch (s) {
            case FIRST:  return SLOT1_SHOOT;
            case SECOND: return SLOT2_SHOOT;
            case THIRD:  return SLOT3_SHOOT;
            default:     return SLOT1_SHOOT;
        }
    }

    private static double sensePosOf(Slot s) {
        switch (s) {
            case FIRST:  return SLOT1_SENSE_POS;
            case SECOND: return SLOT2_SENSE_POS;
            case THIRD:  return SLOT3_SENSE_POS;
            default:     return SLOT1_SENSE_POS;
        }
    }

    private void applyHold(Slot s) {
        setRaw(holdPosOf(s));
    }

    private double computeSettleMs(double targetPos) {
        double est = MS_PER_TRAVEL * Math.abs(targetPos - getTablePos());
        return clamp(est, SETTLE_MIN_MS, SETTLE_MAX_MS);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}

package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import android.graphics.Color;

import java.util.EnumMap;

import static org.firstinspires.ftc.teamcode.util.Constants.*;

public class TurnTable {
    private static final String TURN_SERVO_NAME   = "turntable";
    private static final String HAMMER_SERVO_NAME = "hammer";
    private static final String COLOR_NAME        = "color";

    // ===== 슬롯 =====
    public enum Slot { FIRST, SECOND, THIRD }

    // ===== 감지 색상 =====
    public enum DetectedColor { PURPLE, GREEN, UNKNOWN }


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
    private final NormalizedColorSensor color; // null 허용

    private final EnumMap<Slot, DetectedColor> lastColor = new EnumMap<>(Slot.class);

    private Slot slot = Slot.FIRST;

    private enum State {
        IDLE,
        // Launch
        MOVING, FIRING, RETURNING,
        // nextSlot sensing
        NEXT_MOVING_TO_SENSE, NEXT_SENSING_DWELL, NEXT_MOVING_TO_HOLD
    }
    private State state = State.IDLE;

    private final ElapsedTime timer = new ElapsedTime();
    private double pendingShootPos;
    private Slot   pendingNextSlot;

    private Slot   nextSlotTarget;

    private double currentSettleMs = 0;

    private int sampleCount = 0;
    private float sumH = 0f, sumS = 0f, sumV = 0f;

    public TurnTable(HardwareMap hardwareMap) {
        this.table  = hardwareMap.get(ServoImplEx.class, TURN_SERVO_NAME);
        this.hammer = hardwareMap.get(ServoImplEx.class, HAMMER_SERVO_NAME);

        PwmRange range = new PwmRange(500, 2500);
        this.table.setPwmRange(range);
        this.hammer.setPwmRange(range);

        NormalizedColorSensor c = null;
        try {
            c = hardwareMap.get(NormalizedColorSensor.class, COLOR_NAME);
            try { c.setGain(2.0f); } catch (Exception ignored) {}
        } catch (Exception ignored) {}
        this.color = c;

        this.hammer.setPosition(HAMMER_REST);
        applyHold(slot);

        lastColor.put(Slot.FIRST,  DetectedColor.UNKNOWN);
        lastColor.put(Slot.SECOND, DetectedColor.UNKNOWN);
        lastColor.put(Slot.THIRD,  DetectedColor.UNKNOWN);
    }



    public void setSlot(Slot s) {
        if (s == null || isBusy()) return;
        slot = s;
        applyHold(slot);
    }

    public void nextSlot() {
        if (isBusy()) return;
        nextSlotTarget = next(slot);

        // 센싱 웨이포인트로 이동 (동적 settle 계산)
        double target = sensePosOf(slot); // keep sensing for the NEXT slot
        currentSettleMs = computeSettleMs(target);
        setRaw(target);
        timer.reset();
        state = State.NEXT_MOVING_TO_SENSE;
    }

    public void prevSlot() {
        if (isBusy()) return;
        slot = prev(slot);
        applyHold(slot);
    }


    public void startLaunchCurrent() { startLaunch(slot); }


    public void startLaunch(Slot s) {
        if (state != State.IDLE) return;

        pendingShootPos = shootingPosOf(s);
        pendingNextSlot = next(s);

        setRaw(pendingShootPos);
        timer.reset();
        state = State.MOVING;
        lastColor.put(s, DetectedColor.UNKNOWN);
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

            /* ---------- nextSlot sensing ---------- */
            case NEXT_MOVING_TO_SENSE:
                if (timer.milliseconds() >= currentSettleMs) {
                    sampleCount = 0;
                    sumH = sumS = sumV = 0f;
                    timer.reset();
                    state = State.NEXT_SENSING_DWELL;
                }
                break;

            case NEXT_SENSING_DWELL:
                if (color != null) {
                    try {
                        NormalizedRGBA rgba = color.getNormalizedColors();
                        float r = clamp01(rgba.red), g = clamp01(rgba.green), b = clamp01(rgba.blue);

                        float sum = r + g + b; if (sum < 1e-6f) sum = 1e-6f;
                        float rc = r / sum, gc = g / sum, bc = b / sum;

                        float[] hsv = new float[3];
                        Color.RGBToHSV((int)(rc * 255f), (int)(gc * 255f), (int)(bc * 255f), hsv);

                        sumH += hsv[0];
                        sumS += hsv[1];
                        sumV += hsv[2];
                        sampleCount++;
                    } catch (Exception ignored) {}
                }


                if (sampleCount >= SAMPLE_TARGET || timer.milliseconds() >= SENSE_DWELL_MS) {
                    DetectedColor dc;
                    if (sampleCount > 0) {
                        float inv = 1f / sampleCount;
                        float H = sumH * inv, S = sumS * inv, V = sumV * inv;
                        dc = classifyHSV(H, S, V);
                    } else {
                        dc = DetectedColor.UNKNOWN;
                    }

                    lastColor.put(slot, dc);


                    double targetHold = holdPosOf(nextSlotTarget);
                    currentSettleMs = computeSettleMs(targetHold);
                    setRaw(targetHold);
                    timer.reset();
                    state = State.NEXT_MOVING_TO_HOLD;
                }
                break;

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

    public void nudge(double delta) {
        table.setPosition(Range.clip(table.getPosition() + delta, 0.0, 1.0));
    }

    public void setRaw(double pos) {
        table.setPosition(Range.clip(pos, 0.0, 1.0));
    }

    public Slot   getSlot()      { return slot; }
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
            case FIRST:  return SHOOT_SLOT1;
            case SECOND: return SHOOT_SLOT2;
            case THIRD:  return SHOOT_SLOT3;
            default:     return SHOOT_SLOT1;
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

    private static float clamp01(float v) {
        return v < 0f ? 0f : (v > 1f ? 1f : v);
    }

    private static DetectedColor classifyHSV(float H, float S, float V) {
        if (S < 0.25f || V < 0.10f) return DetectedColor.UNKNOWN;
        if (H >= 90 && H <= 180)   return DetectedColor.GREEN;
        if (H >= 200 && H <= 320)  return DetectedColor.PURPLE;
        return DetectedColor.UNKNOWN;
    }
}

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

import static org.firstinspires.ftc.teamcode.util.Constants.*; // SLOT/HAMMER/TIMING 등 기존 상수 사용

/**
 * TurnTable: 슬롯 기반 포지셔닝 + 런치 시퀀스(해머 포함) + nextSlot() 색 센싱 일시정지.
 * - 생성자 인자: HardwareMap (내부에서 ServoImplEx, ColorSensor 획득)
 * - Servo PWM 범위: 500 ~ 2500 μs
 * - 발사 후 자동으로 다음 슬롯의 hold 위치로 이동
 * - nextSlot(): 센싱 웨이포인트에서 잠시 멈추고 색 감지 후 다음 슬롯 hold로 이동
 *
 * Integrations kept from your TestColorSensor changes:
 *  - setGain(...) only (no LED toggle)
 *  - distance-based dynamic settle time with clamp
 *  - 12-sample accumulation during dwell, with time fallback
 */
public class TurnTable {

    // ===== 하드웨어 이름 =====
    private static final String TURN_SERVO_NAME   = "turntable";
    private static final String HAMMER_SERVO_NAME = "hammer";
    private static final String COLOR_NAME        = "color";

    // ===== 슬롯 =====
    public enum Slot { FIRST, SECOND, THIRD }

    // ===== 감지 색상 =====
    public enum DetectedColor { PURPLE, GREEN, UNKNOWN }

    // ===== 색 센싱 웨이포인트/타이밍 =====
    private static final double SLOT1_SENSE_POS = 0.32;
    private static final double SLOT2_SENSE_POS = 0.67;
    private static final double SLOT3_SENSE_POS = 1.00;
    private static final double SENSE_DWELL_MS  = 400;  // 정지 후 샘플링 시간

    // === 동적 settle 계산 파라미터 ===
    private static final double MS_PER_TRAVEL = 800; // pos 1.0 이동당 대략 800ms (기체에 맞게 튜닝)
    private static final double SETTLE_MIN_MS = 150; // 하한
    private static final double SETTLE_MAX_MS = 900; // 상한

    // === 샘플링 설정 ===
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

    // 동적 settle 저장
    private double currentSettleMs = 0;

    // --- 센싱 누적(HSV 평균용) ---
    private int sampleCount = 0;
    private float sumH = 0f, sumS = 0f, sumV = 0f;

    public TurnTable(HardwareMap hardwareMap) {
        this.table  = hardwareMap.get(ServoImplEx.class, TURN_SERVO_NAME);
        this.hammer = hardwareMap.get(ServoImplEx.class, HAMMER_SERVO_NAME);

        // PWM 범위 500~2500 μs 설정
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

    /* ------------------------- 공개 API ------------------------- */

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

    /** 현재 슬롯 기준으로 런치 시작 */
    public void startLaunchCurrent() { startLaunch(slot); }

    /** 지정 슬롯을 슈팅 → 해머 펄스 → 다음 슬롯 Hold로 자동 전환 */
    public void startLaunch(Slot s) {
        if (state != State.IDLE) return;

        pendingShootPos = shootingPosOf(s);  // Constants에서 가져옴
        pendingNextSlot = next(s);

        setRaw(pendingShootPos);
        timer.reset();
        state = State.MOVING;
        lastColor.put(s, DetectedColor.UNKNOWN);
    }

    /** 색상 기준 최단 이동 슬롯 선택 후 발사 */
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

    /** OpMode.loop()에서 매 프레임 호출 */
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
                    // 센싱 준비: 누적 초기화
                    sampleCount = 0;
                    sumH = sumS = sumV = 0f;
                    timer.reset();
                    state = State.NEXT_SENSING_DWELL;
                }
                break;

            case NEXT_SENSING_DWELL:
                // 매 프레임 샘플 취득 시도
                if (color != null) {
                    try {
                        NormalizedRGBA rgba = color.getNormalizedColors();
                        float r = clamp01(rgba.red), g = clamp01(rgba.green), b = clamp01(rgba.blue);
                        // 크로마틱 정규화 → HSV
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

                // 종료 조건: 목표 샘플 도달 or 시간 만료
                if (sampleCount >= SAMPLE_TARGET || timer.milliseconds() >= SENSE_DWELL_MS) {
                    DetectedColor dc;
                    if (sampleCount > 0) {
                        float inv = 1f / sampleCount;
                        float H = sumH * inv, S = sumS * inv, V = sumV * inv;
                        dc = classifyHSV(H, S, V);
                    } else {
                        dc = DetectedColor.UNKNOWN;
                    }
                    // 기록
                    lastColor.put(slot, dc);

                    // 다음 슬롯 Hold로 이동 (동적 settle 재계산)
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

    /** 마지막으로 기록된 슬롯별 색상 */
    public DetectedColor getLastColor(Slot s) {
        DetectedColor dc = lastColor.get(s);
        return dc == null ? DetectedColor.UNKNOWN : dc;
    }

    /** 수동 미세조정(슬롯 enum은 유지, 물리 위치만 이동) */
    public void nudge(double delta) {
        table.setPosition(Range.clip(table.getPosition() + delta, 0.0, 1.0));
    }

    /** 즉시 원시 위치로 이동(0..1) */
    public void setRaw(double pos) {
        table.setPosition(Range.clip(pos, 0.0, 1.0));
    }

    public Slot   getSlot()      { return slot; }
    public double getTablePos()  { return table.getPosition(); }
    public boolean isBusy()      { return state != State.IDLE; }

    /* ------------------------- 내부 유틸 ------------------------- */

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
            case FIRST:  return SLOT1_HOLD;  // Constants에서 제공
            case SECOND: return SLOT2_HOLD;
            case THIRD:  return SLOT3_HOLD;
            default:     return SLOT1_HOLD;
        }
    }

    private static double shootingPosOf(Slot s) {
        switch (s) {
            case FIRST:  return SHOOT_SLOT1; // Constants에서 제공
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

    /** HSV 기반 간단 분류 (필요 시 Constants로 이동 가능) */
    private static DetectedColor classifyHSV(float H, float S, float V) {
        if (S < 0.25f || V < 0.10f) return DetectedColor.UNKNOWN;
        if (H >= 90 && H <= 180)   return DetectedColor.GREEN;
        if (H >= 200 && H <= 320)  return DetectedColor.PURPLE;
        return DetectedColor.UNKNOWN;
    }
}

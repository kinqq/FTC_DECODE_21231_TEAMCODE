package org.firstinspires.ftc.teamcode.test;

import android.graphics.Color;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TestColorDetection", group = "Test")
public class TestColorDetection extends OpMode {

    // --- 하드웨어 이름(필요시 변경) ---
    private static final String TURN_SERVO_NAME = "turntable";
    private static final String COLOR_NAME      = "color";

    // --- Servo 설정 ---
    private static final PwmRange SERVO_RANGE = new PwmRange(500, 2500);

    // --- 슬롯/센싱 포지션 ---
    private static final double[] SENSE_POS = { 0.22, 0.57, 0.91 }; // slot1/2/3 sensing
    private int slotIdx = 0; // 0..2

    // --- 타이밍 (ms) ---
    private static double MOVE_SETTLE_MS = 400; // 웨이포인트 도달 안정화
    private static final double DWELL_MS       = 400; // 정지 후 샘플링 시간

    // --- 샘플링 설정 ---
    private static final int SAMPLE_TARGET = 12; // dwell 동안 모을 목표 샘플 수

    // --- 감지 결과 저장 ---
    private enum DetectedColor { PURPLE, GREEN, UNKNOWN }
    private DetectedColor[] lastColor = { DetectedColor.UNKNOWN, DetectedColor.UNKNOWN, DetectedColor.UNKNOWN };

    // 최근 평균값 저장 (디스플레이용)
    private static class Avg {
        float r, g, b, a;   // normalized RGBA (0..1)
        float rc, gc, bc;   // chromatic RGB (조명정규화)
        float H, S, V;      // HSV
        void reset() { r=g=b=a=rc=gc=bc=H=S=V=0f; }
    }
    private final Avg[] lastAvg = { new Avg(), new Avg(), new Avg() };

    // --- 하드웨어 ---
    private ServoImplEx turn;
    private DcMotor motor;
    private NormalizedColorSensor color;

    // --- 상태머신 ---
    private enum State { IDLE, MOVING, DWELLING }
    private State state = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();

    // 누적용
    private int sampleCount = 0;
    private float sumR, sumG, sumB, sumA, sumRC, sumGC, sumBC, sumH, sumS, sumV;

    private boolean prevLB = false;

    @Override
    public void init() {
        HardwareMap hw = hardwareMap;

        turn = hw.get(ServoImplEx.class, TURN_SERVO_NAME);
        turn.setPwmRange(SERVO_RANGE);
        turn.setPosition(SENSE_POS[0]);
        motor = hw.get(DcMotor.class, "intake");
        motor.setPower(-1);

        try {
            color = hw.get(NormalizedColorSensor.class, COLOR_NAME);
            color.setGain(500);
        } catch (Exception e) {
            color = null; // 센서 미장착 허용
        }

        // 시작 시 현재 슬롯의 센싱 포지션으로 이동 후 IDLE
        double target = SENSE_POS[slotIdx];
        MOVE_SETTLE_MS = 1000 * Math.abs(target - turn.getPosition());
        turn.setPosition(Range.clip(target, 0.0, 1.0));

        state = State.MOVING;
        timer.reset();

        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());


        telemetry.addLine("TestColorSensor Ready");
        telemetry.addLine("LB: next slot → move & dwell → sample & classify");
        telemetry.update();
    }

    @Override
    public void loop() {
        // LB edge → 다음 슬롯 이동 시작
        boolean lb = gamepad1.left_bumper;
        if (lb && !prevLB && state == State.IDLE) {
            slotIdx = (slotIdx + 1) % 3;

            double target = SENSE_POS[slotIdx];
            MOVE_SETTLE_MS = 1000 * Math.abs(target - turn.getPosition());
            turn.setPosition(Range.clip(target, 0.0, 1.0));
            state = State.MOVING;
            timer.reset();
        }
        prevLB = lb;

        // 상태 진행
        switch (state) {
            case MOVING:
                if (timer.milliseconds() >= MOVE_SETTLE_MS) {
                    // dwell 시작: 누적 리셋
                    resetAccum();
                    state = State.DWELLING;
                    timer.reset();
                }
                break;

            case DWELLING:
                if (color != null) {
                    // 샘플 취득
                    NormalizedRGBA rgba = color.getNormalizedColors();
                    float r=cl01(rgba.red), g=cl01(rgba.green), b=cl01(rgba.blue), a=cl01(rgba.alpha);
                    float sum = r+g+b; if (sum < 1e-6f) sum = 1e-6f;
                    float rc = r/sum, gc = g/sum, bc = b/sum;

                    float[] hsv = new float[3];
                    Color.RGBToHSV((int)(rc*255f), (int)(gc*255f), (int)(bc*255f), hsv);
                    float H=hsv[0], S=hsv[1], V=hsv[2];

                    // 누적
                    sumR+=r; sumG+=g; sumB+=b; sumA+=a;
                    sumRC+=rc; sumGC+=gc; sumBC+=bc;
                    sumH+=H; sumS+=S; sumV+=V;
                    sampleCount++;
                }

                // dwell 종료 조건(시간 또는 충분 샘플)
//                if (timer.milliseconds() >= DWELL_MS || sampleCount >= SAMPLE_TARGET) {
                if (sampleCount >= SAMPLE_TARGET) {
                        // 평균 계산 & 분류 저장
                    Avg avg = lastAvg[slotIdx];
                    if (sampleCount > 0) {
                        float inv = 1f / sampleCount;
                        avg.r = sumR*inv; avg.g = sumG*inv; avg.b = sumB*inv; avg.a = sumA*inv;
                        avg.rc = sumRC*inv; avg.gc = sumGC*inv; avg.bc = sumBC*inv;
                        avg.H = sumH*inv; avg.S = sumS*inv; avg.V = sumV*inv;
                        lastColor[slotIdx] = classify(avg.H, avg.S, avg.V);
                    } else {
                        avg.reset();
                        lastColor[slotIdx] = DetectedColor.UNKNOWN;
                    }
                    state = State.IDLE;
                }
                break;

            case IDLE:
                // no-op
                break;
        }

        // 텔레메트리
        telemetry.addLine("=== TestColorSensor (Tuning) ===");
        telemetry.addData("State", state);
        telemetry.addData("Slot", slotIdx+1);
        telemetry.addData("TargetSensePos", "%.3f", SENSE_POS[slotIdx]);
        telemetry.addLine();

        for (int i = 0; i < 3; i++) {
            Avg a = lastAvg[i];
            telemetry.addLine(String.format("-- Slot %d --", i+1));
            telemetry.addData("LastColor", lastColor[i]);
            telemetry.addData("HSV", "H=%.1f  S=%.2f  V=%.2f", a.H, a.S, a.V);
            telemetry.addData("RGB(n)", "r=%.3f g=%.3f b=%.3f", a.r, a.g, a.b);
            telemetry.addData("RGB(c)", "rc=%.3f gc=%.3f bc=%.3f", a.rc, a.gc, a.bc);
            telemetry.addLine();
        }
        telemetry.addLine("LB: Next slot → move→dwell→sample");
        telemetry.update();
    }

    /* ---------- 유틸 ---------- */

    private void resetAccum() {
        sampleCount = 0;
        sumR=sumG=sumB=sumA=sumRC=sumGC=sumBC=sumH=sumS=sumV=0f;
    }

    private static float cl01(float v) { return v < 0f ? 0f : (v > 1f ? 1f : v); }

    /** HSV에 기반한 간단 분류 (현장 캘리브로 조정 권장) */
    private static DetectedColor classify(float H, float S, float V) {
        if (H >= 90 && H <= 180)       return DetectedColor.GREEN;
        if (H >= 200 && H <= 320)      return DetectedColor.PURPLE;
        return DetectedColor.UNKNOWN;
    }
}

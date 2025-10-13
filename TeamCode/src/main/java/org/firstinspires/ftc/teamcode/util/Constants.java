package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

import dev.frozenmilk.sinister.loading.Pinned;
import dev.frozenmilk.sinister.loading.Preload;

@Configurable
@Pinned
public class Constants {
    public static double INTAKE_ACTIVE_POWER = 1;
    public static double INTAKE_IDLE_POWER = 0.3;

    // ===== 턴테이블 포지션 맵 =====
    // Hold(대기) 위치
    public static double SLOT1_HOLD = 0.20;
    public static double SLOT2_HOLD = 0.55;
    public static double SLOT3_HOLD = 0.88;

    // Shooting(발사) 위치
    public static double SHOOT_SLOT1 = 0.71;
    public static double SHOOT_SLOT2 = 0.03;
    public static double SHOOT_SLOT3 = 0.37;

    // ===== 해머 동작 =====
    public static double HAMMER_REST = 0.48;
    public static double HAMMER_FIRE = 0.66;

    // ===== 타이밍 (필요시 조정) =====
    public static double SETTLE_MS = 400; // 슈팅 위치 도달 안정화
    public static double FIRE_MS   = 400; // 해머 전진 펄스
    public static double RESET_MS  = 150; // 해머 복귀 대기
}

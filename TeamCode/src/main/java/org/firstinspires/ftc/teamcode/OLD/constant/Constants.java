package org.firstinspires.ftc.teamcode.OLD.constant;

import com.bylazar.configurables.annotations.Configurable;

import dev.frozenmilk.sinister.loading.Pinned;

@Configurable
@Pinned
public class Constants {
    public static double INTAKE_ACTIVE_POWER = 1;
    public static double INTAKE_IDLE_POWER = 0.3;

    // ===== 턴테이블 포지션 맵 =====
    // Hold(대기) 위치
    public static double SLOT1_HOLD = 0.06;
    public static double SLOT2_HOLD = 0.42;
    public static double SLOT3_HOLD = 0.78;

    // Shooting(발사) 위치
    public static double SLOT1_SHOOT = 0.6;
    public static double SLOT2_SHOOT = 0.92;
    public static double SLOT3_SHOOT = 0.24;

    // ===== 해머 동작 =====
    public static double HAMMER_REST = 0.65;
    public static double HAMMER_FIRE = 0.45;

    // ===== 타이밍 (필요시 조정) =====
    public static double SETTLE_MS = 500; // 슈팅 위치 도달 안정화
    public static double FIRE_MS   = 200; // 해머 전진 펄스
    public static double RESET_MS  = 150; // 해머 복귀 대기

    public static double RED_GOAL_X = 70;
    public static double RED_GOAL_Y = 70;

    public static double BLUE_GOAL_X = -70;
    public static double BLUE_GOAL_Y = 70;
}



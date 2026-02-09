package org.firstinspires.ftc.teamcode.OLD.constant;

import com.bylazar.configurables.annotations.Configurable;

import dev.frozenmilk.sinister.loading.Pinned;

@Configurable
@Pinned
public class Constants {
    // Intake
    public static double INTAKE_ACTIVE_POWER = 1.0;
    public static double INTAKE_IDLE_POWER = 0.3;
    public static double INTAKE_OFF_POWER = 0.0;
    public static double INTAKE_REVERSE_PULSE_SEC = 0.2;
    public static double INTAKE_FORWARD_PULSE_SEC = 0.1;

    // Magazine (turntable + hammer + color)
    public static double MAGAZINE_SERVO_OFFSET = 0.518;
    public static double MAGAZINE_SLOT_FIRST_POS = 0.1;
    public static double MAGAZINE_SLOT_SECOND_POS = 0.273;
    public static double MAGAZINE_SLOT_THIRD_POS = 0.445;

    public static double MAGAZINE_TARGET_FIRST_DEG = 176.94;
    public static double MAGAZINE_TARGET_SECOND_DEG = 52.58;
    public static double MAGAZINE_TARGET_THIRD_DEG = 299.78;
    public static double MAGAZINE_SLOT_MATCH_EPS = 0.001;
    public static double MAGAZINE_BUSY_TOLERANCE_DEG = 10.0;

    public static double MAGAZINE_ANALOG_MAX_VOLT = 3.3;
    public static int MAGAZINE_PWM_MIN_US = 500;
    public static int MAGAZINE_PWM_MAX_US = 2500;

    public static double MAGAZINE_SET_SLOT_STALL_CHECK_SEC = 0.5;
    public static double MAGAZINE_SET_SLOT_STAGE_HOLD_SEC = 0.2;
    public static double MAGAZINE_SET_SLOT_STALL_VEL_THRESHOLD = 100.0;
    public static double MAGAZINE_SET_SLOT_TIMEOUT_SEC = 5.0;
    public static double MAGAZINE_LOCK_TIMEOUT_SEC = 2.0;

    public static double MAGAZINE_HAMMER_UP_POS = 0.45;
    public static double MAGAZINE_HAMMER_DOWN_POS = 0.63;
    public static double MAGAZINE_HAMMER_MOVE_SEC = 0.1;

    public static double MAGAZINE_DISTANCE_INDEX_MM = 44.0;
    public static double MAGAZINE_DISTANCE_INDEX_TIMEOUT_SEC = 0.4;
    public static double MAGAZINE_WAIT_ANY_ARTIFACT_MM = 32.0;
    public static double MAGAZINE_WAIT_ANY_ARTIFACT_TIMEOUT_SEC = 2.0;

    public static double MAGAZINE_INDEX_EMPTY_HUE_MIN = 142.0;
    public static double MAGAZINE_INDEX_EMPTY_HUE_MAX = 147.0;
    public static double MAGAZINE_INDEX_EMPTY_SAT_MIN = 0.45;
    public static double MAGAZINE_INDEX_EMPTY_SAT_MAX = 0.48;

    public static double MAGAZINE_INDEX_GREEN_HUE_MIN = 142.0;
    public static double MAGAZINE_INDEX_GREEN_HUE_MAX = 150.0;
    public static double MAGAZINE_INDEX_GREEN_SAT_MIN = 0.47;
    public static double MAGAZINE_INDEX_GREEN_SAT_MAX = 0.61;

    public static double MAGAZINE_INDEX_PURPLE_HUE_MIN = 142.0;
    public static double MAGAZINE_INDEX_PURPLE_SAT_MIN = 0.34;
    public static double MAGAZINE_INDEX_PURPLE_SAT_MAX = 0.4689;
    public static double MAGAZINE_INDEX_DETECTION_HOLD_SEC = 0.05;

    // Turret + launcher
    public static double TURRET_MOTOR_TO_TURRET_GEAR_RATIO = 5.6111111111;
    public static double TURRET_TICKS_PER_REV = 537.7;
    public static double TURRET_RUN_TO_POSITION_POWER = 1.0;
    public static double TURRET_MIN_DEG = -90.0;
    public static double TURRET_MAX_DEG = 135.0;
    public static double TURRET_COMMAND_DONE_TOLERANCE_DEG = 1.0;

    public static double LAUNCH_ANGLE_MIN_DEG = 15.0;
    public static double LAUNCH_ANGLE_MAX_DEG = 60.0;
    public static double LAUNCH_ANGLE_SERVO_BASE_POS = 0.815;
    public static double LAUNCH_ANGLE_SERVO_PER_DEG = 0.003;

    public static int LAUNCH_ANGLE_PWM_MIN_US = 500;
    public static int LAUNCH_ANGLE_PWM_MAX_US = 2500;
    public static double LAUNCHER_DEFAULT_VELOCITY = 1800.0;
    public static double LAUNCHER_AT_SPEED_TOLERANCE = 30.0;
    public static double LAUNCHER_SPINUP_TIMEOUT_SEC = 3.0;
    public static int LAUNCH_INTER_SHOT_WAIT_MS = 200;

    // Limelight
    public static final int LIMELIGHT_TAG_GPP = 21;
    public static final int LIMELIGHT_TAG_PGP = 22;
    public static final int LIMELIGHT_TAG_PPG = 23;
    public static double LIMELIGHT_WAIT_MOTIF_TIMEOUT_SEC = 4.0;
}

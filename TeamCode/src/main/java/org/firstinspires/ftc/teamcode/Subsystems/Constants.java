package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;

import dev.frozenmilk.sinister.loading.Pinned;

@Configurable
@Pinned
public class Constants
{
    public static double SERVO_P = 0.0006;
    public static double SERVO_I = 0;
    public static double SERVO_D = 0.0000055;

    public static double HOME_P = 0.0009;
    public static double HOME_I = 0;
    public static double HOME_D = 0.0007;
}

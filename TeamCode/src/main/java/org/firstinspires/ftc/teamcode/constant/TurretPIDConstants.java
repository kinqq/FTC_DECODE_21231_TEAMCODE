package org.firstinspires.ftc.teamcode.constant;


import com.bylazar.configurables.annotations.Configurable;

import dev.frozenmilk.sinister.loading.Pinned;

@Configurable
@Pinned
public class TurretPIDConstants
{
    public static double p = 0.02;
    public static double i = 0.0;
    public static double d = 0.0005;
}

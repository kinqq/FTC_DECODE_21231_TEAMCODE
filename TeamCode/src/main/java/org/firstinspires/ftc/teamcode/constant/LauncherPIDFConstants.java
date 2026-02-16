package org.firstinspires.ftc.teamcode.constant;

import com.bylazar.configurables.annotations.Configurable;

import dev.frozenmilk.sinister.loading.Pinned;

@Configurable
@Pinned
public class LauncherPIDFConstants
{
    public static double p = 0.001;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0.000388;
}

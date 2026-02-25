package org.firstinspires.ftc.teamcode.constant;


import com.bylazar.configurables.annotations.Configurable;

import dev.frozenmilk.sinister.loading.Pinned;

@Configurable
@Pinned

public class PTOPIDFConstants
{
    public static double leftP = 0;
    public static double leftI = 0;
    public static double leftD = 0;
    public static double leftF = 12;

    public static double rightP = 0;
    public static double rightI = 0;
    public static double rightD = 0;
    public static double rightF = 12.2;

    public static double frontLP = 0;
    public static double frontLI = 0;
    public static double frontLD = 0;
    public static double frontLF = 12.2;

    public static double frontRP = 0;
    public static double frontRI = 0;
    public static double frontRD = 0;
    public static double frontRF = 12.2;
}

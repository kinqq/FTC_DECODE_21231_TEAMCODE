package org.firstinspires.ftc.teamcode.constant;

import com.bylazar.configurables.annotations.Configurable;

import dev.frozenmilk.sinister.loading.Pinned;

@Configurable
@Pinned
public class ConstantsServo {
    public static double kP = 0.00003;
    public static double kI = 0.00;
    public static double kD = 0.0003;
    public static double targetDeg = 0; // for testing
}

package org.firstinspires.ftc.teamcode.constant;

import com.bylazar.configurables.annotations.Configurable;

import dev.frozenmilk.sinister.loading.Pinned;

@Configurable
@Pinned
public class ConstantsServo {
    public static double kP = 0.0001;
    public static double kI = 0.005;
    public static double kD = 0.000006;
}

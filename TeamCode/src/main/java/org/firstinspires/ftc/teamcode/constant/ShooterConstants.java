package org.firstinspires.ftc.teamcode.constant;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import dev.frozenmilk.sinister.loading.Pinned;

//@Pinned
@Configurable

public class ShooterConstants
{
    //TODO: TUNE PASS THROUGH POINT AND TUNE MAX/MIN ANGLE

    public static Pose GOAL_POS_RED = new Pose(138, 138);
    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();
    public static double SCORE_HEIGHT = 34; //TUNE
    public static double SCORE_ANGLE = Math.toRadians(-45); //TUNE
    public static double PASS_THROUGH_POINT_RADIUS = 10; //TUNE

    public static double HOOD_MAX_ANGLE = Math.toRadians(46); //TUNE
    public static double HOOD_MIN_ANGLE = Math.toRadians(22);
}

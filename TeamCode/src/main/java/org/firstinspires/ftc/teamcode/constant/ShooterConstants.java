package org.firstinspires.ftc.teamcode.constant;

import com.pedropathing.geometry.Pose;

public class ShooterConstants
{
    public static Pose GOAL_POS_RED = new Pose(138, 138);
    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();
    public static double SCORE_HEIGHT = 26;
    public static double SCORE_ANGLE = Math.toRadians(-30);
    public static double PASS_THROUGH_POINT_RADIUS = 5;
}

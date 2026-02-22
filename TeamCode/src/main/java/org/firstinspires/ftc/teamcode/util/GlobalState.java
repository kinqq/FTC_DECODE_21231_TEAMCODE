package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.constant.AllianceColor;
import org.firstinspires.ftc.teamcode.constant.Motif;

public class GlobalState {
    public static Pose teleOpStartPose = null;
    public static Motif lastMotif = Motif.UNKNOWN;
    public static AllianceColor alliance = null;
}

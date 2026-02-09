package org.firstinspires.ftc.teamcode.OLD.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.OLD.constant.AllianceColor;
import org.firstinspires.ftc.teamcode.OLD.subsystem.commands.LimelightCommands.Motif;

public class GlobalState {
    public static Pose teleOpStartPose = null;
    public static Motif lastMotif = Motif.UNKNOWN;
    public static AllianceColor alliance = null;
}

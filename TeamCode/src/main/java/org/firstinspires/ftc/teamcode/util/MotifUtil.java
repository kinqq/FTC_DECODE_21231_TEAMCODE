package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.subsystem.commands.LimelightCommands;

public class MotifUtil {
    public static DetectedColor[] motifToColors(LimelightCommands.Motif motif) {
        if (motif == LimelightCommands.Motif.GPP) {
            return new DetectedColor[]{DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.PURPLE};
        }
        if (motif == LimelightCommands.Motif.PGP) {
            return new DetectedColor[]{DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE};
        }
        if (motif == LimelightCommands.Motif.PPG){
            return new DetectedColor[]{DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN};
        }
        return null;
    }
}

package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.constant.Slot;
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

    public Slot[] pickSlotsForMotif(DetectedColor[] motifColors, DetectedColor[] slotColors) {
        Slot[] slotEnum = new Slot[]{Slot.FIRST, Slot.SECOND, Slot.THIRD};
        Slot[] result = new Slot[3];

        for (int i = 0; i < 3; i++) {
            DetectedColor needed = motifColors[i];
            Slot chosen = null;

            for (int j = 0; j < 3; j++) {
                if (slotColors[j] == needed) {
                    chosen = slotEnum[j];

                    slotColors[j] = DetectedColor.UNKNOWN;
                    break;
                }
            }

            result[i] = chosen;
        }

        return result;
    }
}

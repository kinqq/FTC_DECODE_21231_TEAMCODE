package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.constant.AllianceColor;

@Autonomous(name = "NineArtifactsBlue (Command)")
public class NineArtifactsBlueCmd extends NineArtifactsBaseCmd {
    @Override
    protected AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }
}

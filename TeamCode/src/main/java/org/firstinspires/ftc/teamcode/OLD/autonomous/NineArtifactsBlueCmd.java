package org.firstinspires.ftc.teamcode.OLD.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.OLD.constant.AllianceColor;

@Autonomous(name = "NineArtifactsBlue (Command)", preselectTeleOp = "Drive Meet 2")
public class NineArtifactsBlueCmd extends NineArtifactsBaseCmd {
    @Override
    protected AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }
}

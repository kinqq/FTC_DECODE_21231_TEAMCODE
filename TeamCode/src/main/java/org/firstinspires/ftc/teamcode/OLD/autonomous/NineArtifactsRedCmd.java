package org.firstinspires.ftc.teamcode.OLD.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.OLD.constant.AllianceColor;

@Autonomous(name = "NineArtifactsRed (Command)", preselectTeleOp = "Drive Meet 2")
public class NineArtifactsRedCmd extends NineArtifactsBaseCmd {
    @Override
    protected AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }
}

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.constant.AllianceColor;

@Autonomous(name = "NineArtifactsRed (Command)", preselectTeleOp = "Drive Meet 2")
public class NineArtifactsRedCmd extends NineArtifactsBaseCmd {
    @Override
    protected AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }
}

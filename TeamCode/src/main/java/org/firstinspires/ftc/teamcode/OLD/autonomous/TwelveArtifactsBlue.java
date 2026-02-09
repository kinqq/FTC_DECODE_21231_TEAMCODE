package org.firstinspires.ftc.teamcode.OLD.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OLD.constant.AllianceColor;

@Autonomous(name = "[BLUE] 1\uFE0F⃣ 2\uFE0F⃣ Artifacts", preselectTeleOp = "Drive Meet 2")
public class TwelveArtifactsBlue extends TwelveArtifactsBaseCmd{
    @Override
    protected AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }
}
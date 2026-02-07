package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constant.AllianceColor;

@Autonomous(name = "[RED] 1\uFE0F⃣ 2\uFE0F⃣ Artifacts", preselectTeleOp = "Drive Meet 2")
public class TwelveArtifactsRed extends TwelveArtifactsBaseCmd{
    @Override
    protected AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }
}
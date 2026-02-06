package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constant.AllianceColor;

@Autonomous(name = "TwelveArtifactsBlue", preselectTeleOp = "Drive Meet 2")
public class TwelveArtifactsBlue extends TwelveArtifactsBaseCmd{
    @Override
    protected AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }
}
package org.firstinspires.ftc.teamcode.autonomous.fifteen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constant.AllianceColor;

@Autonomous(name = "[RED] 1\uFE0F⃣5\uFE0F⃣ Artifacts", preselectTeleOp = "DriveSTATE")
public class Red15 extends Base15 {
    @Override
    protected AllianceColor getAllianceColor() { return AllianceColor.RED; }
}

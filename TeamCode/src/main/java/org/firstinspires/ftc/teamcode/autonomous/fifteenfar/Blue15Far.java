package org.firstinspires.ftc.teamcode.autonomous.fifteenfar;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constant.AllianceColor;

@Autonomous(name = "[BLUE] 1\uFE0F⃣5\uFE0F⃣ Artifacts FAR", preselectTeleOp = "DriveSTATE")
public class Blue15Far extends Base15Far{
    @Override
    protected AllianceColor getAllianceColor() { return AllianceColor.BLUE; }
}

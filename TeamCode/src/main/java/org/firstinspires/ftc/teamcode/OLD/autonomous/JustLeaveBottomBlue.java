package org.firstinspires.ftc.teamcode.OLD.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OLD.constant.AllianceColor;

@Autonomous(name = "Just Leave Bottom BLUE", preselectTeleOp = "Drive Meet 2")
public class JustLeaveBottomBlue extends JustLeaveBaseCmd {

    @Override
    public Pose getStart() {return new Pose(64, 5);}

    @Override
    public Pose getEnd() {return new Pose(37, 5);}

    @Override
    public double getStartH() {return 90;}

    @Override
    public double getEndH() {return 90;}

    @Override
    public AllianceColor getAllianceColor() {return AllianceColor.BLUE;}


}

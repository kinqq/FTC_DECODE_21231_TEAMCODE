package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constant.AllianceColor;

@Autonomous(name = "Just Leave Bottom BLUE", preselectTeleOp = "Drive Meet 2")
public class JustLeaveTopClassifierBlue extends JustLeaveBaseCmd {

    @Override
    public Pose getStart() {return new Pose(116.8, 130.35);}

    @Override
    public Pose getEnd() {return new Pose(55, 130);}

    @Override
    public double getStartH() {return 141.823;}

    @Override
    public double getEndH() {return 90;}

    @Override
    public AllianceColor getAllianceColor() {return AllianceColor.BLUE;}

}

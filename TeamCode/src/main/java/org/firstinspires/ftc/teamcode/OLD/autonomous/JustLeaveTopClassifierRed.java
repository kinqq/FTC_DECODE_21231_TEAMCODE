package org.firstinspires.ftc.teamcode.OLD.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Just Leave Top RED", preselectTeleOp = "Drive Meet 2")
public class JustLeaveTopClassifierRed extends JustLeaveBaseCmd {

    @Override
    public Pose getStart() {return new Pose(116.8, 130.35);}

    @Override
    public Pose getEnd() {return new Pose(92, 130);}

    @Override
    public double getStartH() {return 38.177;}

    @Override
    public double getEndH() {return 90;}

}

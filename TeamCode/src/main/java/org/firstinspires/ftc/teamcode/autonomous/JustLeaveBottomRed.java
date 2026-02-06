package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Just Leave Bottom RED", preselectTeleOp = "Drive Meet 2")
public class JustLeaveBottomRed extends JustLeaveBaseCmd {

    @Override
    public Pose getStart() {return new Pose(73, 5);}

    @Override
    public Pose getEnd() {return new Pose(106, 5);}

    @Override
    public double getStartH() {return 90;}

    @Override
    public double getEndH() {return 90;}

}

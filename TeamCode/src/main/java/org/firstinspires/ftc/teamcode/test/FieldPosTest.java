package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.vision.opencv.ColorRange.ARTIFACT_GREEN;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.autonomous.Paths;
import org.firstinspires.ftc.teamcode.constant.AllianceColor;
import org.firstinspires.ftc.teamcode.constant.ConstantsServo;
import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.constant.Slot;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.Draw;
import org.firstinspires.ftc.teamcode.subsystem.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.MagazineCommands;
import org.firstinspires.ftc.teamcode.util.GlobalState;

@TeleOp(name = "Field Pos Test")
@Disabled

public class FieldPosTest extends CommandOpMode {
    private Follower follower;

    private final int SHOT_MODES_COUNT = 4;

    @Override
    public void initialize() {

        follower = Constants.createFollower(hardwareMap);

        //Follower Init
        follower.update();
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        follower.setPose(new Pose(72, 72, Math.toRadians(90)));

        //Enable Panels Telemetry
        PanelsConfigurables.INSTANCE.refreshClass(DriveMeet1.class);
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }

    public double xPos = 135;
    public double yPos = 140;

    @Override
    public void run()
    {
        follower.setPose(new Pose(xPos, yPos, 0));
        follower.update();

        if (gamepad1.aWasPressed()) yPos -= 1;
        if (gamepad1.yWasPressed()) yPos += 1;
        if (gamepad1.xWasPressed()) xPos -= 1;
        if (gamepad1.bWasPressed()) xPos += 1;

        Draw.drawDebug(follower);

        telemetry.addData("xPos", xPos);
        telemetry.addData("yPos", yPos);
        telemetry.addData("xPos REAL", follower.getPose().getX());
        telemetry.addData("xPos REAL", follower.getPose().getY());
        telemetry.update();
    }
}
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.constant.AllianceColor;
import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.constant.Slot;
import org.firstinspires.ftc.teamcode.subsystem.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.MagazineCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.TurretCommands;

import java.util.Objects;

@TeleOp(name = "Drive Meet 2")
public class DriveMeet2 extends CommandOpMode
{
    private MagazineCommands indexerCmds;
    private TurretSubsystem turretCmds;
    private DriveCommands driveCmds;

    public static AllianceColor alliance = AllianceColor.RED;
    private boolean autoAim = true;
    private boolean odoDrive = true;
    private DetectedColor[] mosaic = new DetectedColor[3];

    @Override
    public void initialize()
    {
        indexerCmds = new MagazineCommands(hardwareMap);
        turretCmds = new TurretSubsystem(hardwareMap);
        driveCmds = new DriveCommands(hardwareMap);

        indexerCmds.new zero().initialize();

        turretCmds.setLaunchAngle(0.25);
        turretCmds.setLaunchVel(1900);
    }

    @Override
    public void initialize_loop()
    {
        if (gamepad1.backWasPressed()) {turretCmds.zero(); indexerCmds.new zero().initialize();}
        if (gamepad1.bWasPressed()) alliance = AllianceColor.RED;
        if (gamepad1.xWasPressed()) alliance = AllianceColor.BLUE;
        if (gamepad1.startWasPressed()) odoDrive = !odoDrive;
        if (gamepad1.dpadLeftWasPressed()) turretCmds.upOffset();
        if (gamepad1.dpadRightWasPressed()) turretCmds.downOffset();
        if (gamepad1.dpadUpWasPressed()) turretCmds.launchAngleUp();
        if (gamepad1.dpadDownWasPressed()) turretCmds.launchAngleDown();
        if (gamepad1.yWasPressed()) autoAim = !autoAim;


        indexerCmds.new index(1).initialize();
        indexerCmds.new index(2).initialize();
        indexerCmds.new index(3).initialize();

        telemetry.addData("CURRENT POS", indexerCmds.getPos());
        telemetry.addData("ALLIANCE", alliance);
        telemetry.addData("ODO DRIVE", odoDrive);
        telemetry.addData("AUTO AIM", autoAim);
        telemetry.addData("OFFSET", turretCmds.getOffset());
        telemetry.addData("LAUNCH ANGLE", turretCmds.getLaunchAngle());

        telemetry.addLine();
        telemetry.addData("FIRST", indexerCmds.getSlot(Slot.FIRST));
        telemetry.addData("SECOND", indexerCmds.getSlot(Slot.SECOND));
        telemetry.addData("THIRD", indexerCmds.getSlot(Slot.THIRD));

        telemetry.update();
    }

    private ElapsedTime findTimer = new ElapsedTime();
    private boolean started = false;
    @Override
    public void run() {
        super.run();
        if (!started) driveCmds.new intakeOn().initialize(); started = true;


        if (odoDrive) driveCmds.odoDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
        else driveCmds.basicDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

        indexerCmds.update();
        indexerCmds.new index(1).initialize();
        indexerCmds.new index(2).initialize();
        indexerCmds.new index(3).initialize();

        if (indexerCmds.updateDone())
        {
        find(DetectedColor.UNKNOWN);
        }

        if (gamepad1.backWasPressed()) {turretCmds.zero(); indexerCmds.new zero().initialize();}
        if (gamepad1.startWasPressed()) odoDrive = !odoDrive;
        if (gamepad1.yWasPressed()) autoAim = !autoAim;

        if (gamepad2.dpadLeftWasPressed()) turretCmds.upOffset();
        if (gamepad2.dpadRightWasPressed()) turretCmds.downOffset();
        if (gamepad2.dpadUpWasPressed()) turretCmds.launchAngleUp();
        if (gamepad2.dpadDownWasPressed()) turretCmds.launchAngleDown();

        if (gamepad2.yWasPressed() && indexerCmds.updateDone())
        {
            schedule(new SequentialCommandGroup(
                    indexerCmds.new switchMode(),
                    driveCmds.new intakeIdle(),
                    shoot(Slot.FIRST),
                    shoot(Slot.SECOND),
                    shoot(Slot.SECOND),
                    indexerCmds.new switchMode(),
                    driveCmds.new intakeOn(),
                    turretCmds.new spinDown()
            ));
        }

        turretCmds.update();;

        telemetry.addData("FIRST", indexerCmds.getSlot(Slot.FIRST));
        telemetry.addData("SECOND", indexerCmds.getSlot(Slot.SECOND));
        telemetry.addData("THIRD", indexerCmds.getSlot(Slot.THIRD));
        telemetry.addData("TARGET", indexerCmds.getTarget());
        telemetry.addData("CURRENT POS", indexerCmds.getPos());
        telemetry.update();

    }

    private void find(DetectedColor color) {
        switch (color) {
            case GREEN:
                if (indexerCmds.getSlot(Slot.FIRST) == DetectedColor.GREEN)
                    indexerCmds.new setSlot(Slot.FIRST,false).initialize();
                if (indexerCmds.getSlot(Slot.SECOND) == DetectedColor.GREEN)
                    indexerCmds.new setSlot(Slot.SECOND,false).initialize();
                if (indexerCmds.getSlot(Slot.THIRD) == DetectedColor.PURPLE)
                    indexerCmds.new setSlot(Slot.THIRD,false).initialize();
                break;
            case PURPLE:
                if (indexerCmds.getSlot(Slot.FIRST) == DetectedColor.PURPLE)
                    indexerCmds.new setSlot(Slot.FIRST,false).initialize();
                if (indexerCmds.getSlot(Slot.SECOND) == DetectedColor.PURPLE)
                    indexerCmds.new setSlot(Slot.SECOND,false).initialize();
                if (indexerCmds.getSlot(Slot.THIRD) == DetectedColor.PURPLE)
                    indexerCmds.new setSlot(Slot.THIRD,false).initialize();
                break;
            case UNKNOWN:
                if (indexerCmds.getSlot(Slot.FIRST) == DetectedColor.UNKNOWN)
                    indexerCmds.new setSlot(Slot.FIRST,false).initialize();
                else if (indexerCmds.getSlot(Slot.SECOND) == DetectedColor.UNKNOWN)
                    indexerCmds.new setSlot(Slot.SECOND,false).initialize();
                else if (indexerCmds.getSlot(Slot.THIRD) == DetectedColor.UNKNOWN)
                    indexerCmds.new setSlot(Slot.THIRD,false).initialize();
                break;
        }
    }

    private CommandBase shoot(Slot slot) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        indexerCmds.new setSlot(slot,true),
                        turretCmds.new spinUp()
                ),
                indexerCmds.new hammerUp(),
                indexerCmds.new hammerDown()
        );
    }

}
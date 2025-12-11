package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.constant.AllianceColor;
import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.constant.Slot;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.MagazineCommands;
import org.firstinspires.ftc.teamcode.util.GlobalState;

@TeleOp(name = "Drive Meet 2")
public class DriveMeet2 extends CommandOpMode
{
    private MagazineCommands indexerCmds;
    private TurretSubsystem turretCmds;
    private DriveCommands driveCmds;

    private Follower follower;

    public static AllianceColor alliance = AllianceColor.RED;
    private boolean autoAim = true;
    private boolean odoDrive = true;
    private boolean launching = false;
    private LimelightCommands.Motif motif = LimelightCommands.Motif.PPG;
    private DetectedColor[] motifTranslated = new DetectedColor[] {DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN};

    @Override
    public void initialize()
    {
        indexerCmds = new MagazineCommands(hardwareMap);
        turretCmds = new TurretSubsystem(hardwareMap);
        driveCmds = new DriveCommands(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

        indexerCmds.new zero().initialize();

        turretCmds.setLaunchAngle(0.13);
        turretCmds.setLaunchVel(1900);

        if (GlobalState.teleOpStartPose != null) {
            follower.setStartingPose(GlobalState.teleOpStartPose);
        } else {
            follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        }

        if (GlobalState.alliance != null) {
            alliance = GlobalState.alliance;
        }
        else {
            alliance = AllianceColor.RED;
        }

        if (GlobalState.lastMotif != LimelightCommands.Motif.UNKNOWN) {
            motif = GlobalState.lastMotif;
        }
        else {
            motif = LimelightCommands.Motif.PPG;
        }
        follower.update();

        PanelsConfigurables.INSTANCE.refreshClass(DriveMeet1.class);
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        if (motif == LimelightCommands.Motif.PPG)
        {
            motifTranslated[0] = DetectedColor.PURPLE;
            motifTranslated[1] = DetectedColor.PURPLE;
            motifTranslated[2] = DetectedColor.GREEN;
        }
        if (motif == LimelightCommands.Motif.PGP)
        {
            motifTranslated[0] = DetectedColor.PURPLE;
            motifTranslated[1] = DetectedColor.GREEN;
            motifTranslated[2] = DetectedColor.PURPLE;
        }
        if (motif == LimelightCommands.Motif.GPP)
        {
            motifTranslated[0] = DetectedColor.GREEN;
            motifTranslated[1] = DetectedColor.PURPLE;
            motifTranslated[2] = DetectedColor.PURPLE;
        }
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
        if (gamepad1.aWasPressed()) {
            follower.setPose(new Pose(72, 72, Math.toRadians(90)));
        }


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
        if (!started) {
            driveCmds.new intakeOn().initialize();
            started = true; findTimer.reset();
            follower.startTeleopDrive();
        }

        follower.update();

        if (alliance == AllianceColor.BLUE) {
            follower.setTeleOpDrive(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    !odoDrive
            );
        }
        if (alliance == AllianceColor.RED) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    !odoDrive
            );
        }

        indexerCmds.update();

        if (indexerCmds.updateDone() && !launching)
        {
            indexerCmds.new index(1).initialize();
            find(DetectedColor.UNKNOWN);
        }

        if (gamepad1.backWasPressed()) {turretCmds.zero(); indexerCmds.new zero().initialize();}
        if (gamepad1.startWasPressed() && !gamepad1.a) odoDrive = !odoDrive;
        if (gamepad1.yWasPressed()) autoAim = !autoAim;
        if (gamepad2.dpadLeftWasPressed()) turretCmds.upOffset();
        if (gamepad2.dpadRightWasPressed()) turretCmds.downOffset();
        if (gamepad2.dpadUpWasPressed()) turretCmds.launchAngleUp();
        if (gamepad2.dpadDownWasPressed()) turretCmds.launchAngleDown();

        if (gamepad2.yWasPressed() && indexerCmds.updateDone())
        {
            schedule(new SequentialCommandGroup(
                    new CommandBase() {
                        @Override public void initialize() {launching = true;}
                        @Override public boolean isFinished() {return true;}
                    },
                    indexerCmds.new switchMode(),
                    driveCmds.new intakeIdle(),
                    shoot(Slot.FIRST),
                    shoot(Slot.SECOND),
                    shoot(Slot.SECOND),
                    indexerCmds.new switchMode(),
                    driveCmds.new intakeOn(),
                    turretCmds.new spinDown(),
                    new CommandBase() {
                        @Override public void initialize() {launching = false;}
                        @Override public boolean isFinished() {return true;}
                    }
            ));
        }
        if (gamepad2.xWasPressed() && indexerCmds.updateDone())
        {
            schedule(new SequentialCommandGroup(
                    new CommandBase() {
                        @Override public void initialize() {launching = true;}
                        @Override public boolean isFinished() {return true;}
                    },
                    indexerCmds.new switchMode(),
                    driveCmds.new intakeIdle(),
                    new CommandBase() {
                        private CommandBase inner;

                        @Override
                        public void initialize() {
                            Slot[] slots = pickSlotsForMotif(motifTranslated);

                            if (slots[0] == Slot.FIRST && slots[1] == Slot.SECOND) slots[2] = Slot.SECOND;
                            if (slots[0] == Slot.FIRST && slots[1] == Slot.THIRD) slots[2] = Slot.THIRD;
                            if (slots[0] == Slot.SECOND) {
                                if (slots[1] == Slot.FIRST) slots[1] = Slot.THIRD;
                                else if (slots[1] == Slot.THIRD) slots[1] = Slot.SECOND;
                            }
                            if (slots[0] == Slot.SECOND && slots[1] == Slot.THIRD && slots[2] == Slot.FIRST) slots[2] = Slot.SECOND;
                            if (slots[0] == Slot.THIRD) {
                                if (slots[1] == Slot.FIRST) slots[1] = Slot.SECOND;
                                else if (slots[1] == Slot.SECOND) slots[1] = Slot.THIRD;
                            }
                            if (slots[0] == Slot.THIRD && slots[1] == Slot.SECOND) slots[2] = Slot.THIRD;

                            CommandBase c0 = (slots[0] != null) ? shoot(slots[0]) : new InstantCommand(() -> {});
                            CommandBase c1 = (slots[1] != null) ? shoot(slots[1]) : new InstantCommand(() -> {});
                            CommandBase c2 = (slots[2] != null) ? shoot(slots[2]) : new InstantCommand(() -> {});

                            inner = new SequentialCommandGroup(c0, c1, c2);
                            inner.initialize();
                        }

                        @Override
                        public void execute() {
                            if (inner != null) inner.execute();
                        }

                        @Override
                        public boolean isFinished() {
                            return inner == null || inner.isFinished();
                        }

                        @Override
                        public void end(boolean interrupted) {
                            if (inner != null) inner.end(interrupted);
                        }
                    },
                    indexerCmds.new switchMode(),
                    driveCmds.new intakeOn(),
                    turretCmds.new spinDown(),
                    new CommandBase() {
                        @Override public void initialize() {launching = false;}
                        @Override public boolean isFinished() {return true;}
                    }
            ));
        }

        turretCmds.update();;

        telemetry.addData("FIRST", indexerCmds.getSlot(Slot.FIRST));
        telemetry.addData("SECOND", indexerCmds.getSlot(Slot.SECOND));
        telemetry.addData("THIRD", indexerCmds.getSlot(Slot.THIRD));
        telemetry.addData("TARGET", indexerCmds.getTarget());
        telemetry.addData("CURRENT POS", indexerCmds.getPos());
        telemetry.addData("Pos X", driveCmds.getOdo().getPosX(DistanceUnit.MM));
        telemetry.addData("Pos Y", driveCmds.getOdo().getPosY(DistanceUnit.MM));
        telemetry.update();

    }

    private Slot find(DetectedColor color) {
        switch (color) {
            case GREEN:
                if (indexerCmds.getSlot(Slot.FIRST) == DetectedColor.GREEN)
                {
                    //indexerCmds.new setSlot(Slot.FIRST, false).initialize();
                    return Slot.FIRST;
                }
                if (indexerCmds.getSlot(Slot.SECOND) == DetectedColor.GREEN) {
                    //indexerCmds.new setSlot(Slot.SECOND, false).initialize();
                    return Slot.SECOND;
                }
                if (indexerCmds.getSlot(Slot.THIRD) == DetectedColor.PURPLE) {
                    //indexerCmds.new setSlot(Slot.THIRD, false).initialize();
                    return Slot.THIRD;
                }
                break;
            case PURPLE:
                if (indexerCmds.getSlot(Slot.FIRST) == DetectedColor.PURPLE)
                {
                   // indexerCmds.new setSlot(Slot.FIRST,false).initialize();
                    return Slot.FIRST;
                }
                if (indexerCmds.getSlot(Slot.SECOND) == DetectedColor.PURPLE)
                {
                  //  indexerCmds.new setSlot(Slot.SECOND,false).initialize();
                    return Slot.SECOND;
                }
                if (indexerCmds.getSlot(Slot.THIRD) == DetectedColor.PURPLE)
                {
                    indexerCmds.new setSlot(Slot.THIRD).initialize();
                    return Slot.THIRD;
                }
                break;
            case UNKNOWN:
                if (indexerCmds.getSlot(Slot.FIRST) == DetectedColor.UNKNOWN)
                {
                    indexerCmds.new setSlot(Slot.FIRST).initialize();
                    return Slot.FIRST;
                }
                else if (indexerCmds.getSlot(Slot.SECOND) == DetectedColor.UNKNOWN)
                {
                    indexerCmds.new setSlot(Slot.SECOND).initialize();
                    return Slot.SECOND;
                }
                else if (indexerCmds.getSlot(Slot.THIRD) == DetectedColor.UNKNOWN)
                {
                    indexerCmds.new setSlot(Slot.THIRD).initialize();
                    return Slot.THIRD;
                }
                break;
        }
        return Slot.FIRST;
    }

    private CommandBase shoot(Slot slot) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        indexerCmds.new setSlot(slot),
                        turretCmds.new spinUp()
                ),
                indexerCmds.new hammerUp(),
                indexerCmds.new hammerDown(),
                indexerCmds.new clearFirst()
        );
    }

    private Slot[] pickSlotsForMotif(DetectedColor[] motifColors) {
        DetectedColor[] slotColors = new DetectedColor[]{
                indexerCmds.getSlot(Slot.FIRST),
                indexerCmds.getSlot(Slot.SECOND),
                indexerCmds.getSlot(Slot.THIRD)
        };
        Slot[] slotEnum = new Slot[]{Slot.FIRST, Slot.SECOND, Slot.THIRD};

        Slot[] result = new Slot[3];

        for (int i = 0; i < 3; i++) {
            DetectedColor needed = motifColors[i];
            Slot chosen = null;

            for (int j = 0; j < 3; j++) {
                if (slotColors[j] == needed) {
                    chosen = slotEnum[j];

                    slotColors[j] = DetectedColor.UNKNOWN;
                    break;
                }
            }

            result[i] = chosen;
        }

        return result;
    }

}
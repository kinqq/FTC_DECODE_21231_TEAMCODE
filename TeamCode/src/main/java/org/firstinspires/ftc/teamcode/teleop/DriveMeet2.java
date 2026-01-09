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
import org.firstinspires.ftc.teamcode.Subsystems.*;
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

@TeleOp(name = "Drive Meet 2")
public class DriveMeet2 extends CommandOpMode
{
    //Subsystems
    private MagazineCommands indexerCmds;
    private TurretSubsystem turretCmds;
    private DriveCommands driveCmds;
    private Follower follower;

    //General Info
    private boolean autoAim = true;
    private boolean odoDrive = true;
    private boolean launching = false;
    public static AllianceColor alliance = AllianceColor.RED;

    private DetectedColor[] motifTranslated = new DetectedColor[] {DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN};

    @Override
    public void initialize()
    {
        //Subsystem Init
        indexerCmds = new MagazineCommands(hardwareMap);
        turretCmds = new TurretSubsystem(hardwareMap);
        driveCmds = new DriveCommands(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        //Indexer Init
        indexerCmds.new zero().initialize();

        //Turret Init
        turretCmds.setLaunchAngle(0.13);
        turretCmds.setLaunchVel(1900);

        //Follower Init
        if (GlobalState.teleOpStartPose != null) {
            follower.setStartingPose(new Pose(GlobalState.teleOpStartPose.getPose().getX(), GlobalState.teleOpStartPose.getPose().getY(), Math.toRadians(45)));
        } else {
            follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        }
        follower.update();

        //Gather Alliance Info
        if (GlobalState.alliance != null) {
            alliance = GlobalState.alliance;
        }
        else {
            alliance = AllianceColor.RED;
        }

        //Gather Motif Data
        //Motif Info
        LimelightCommands.Motif motif;

        if (GlobalState.lastMotif != LimelightCommands.Motif.UNKNOWN) {
            motif = GlobalState.lastMotif;
        }
        else {
            motif = LimelightCommands.Motif.PPG;
        }

        //Translate Motif
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

        indexerCmds.new hammerDown().initialize();
        indexerCmds.update();

        //Enable Panels Telemetry
        PanelsConfigurables.INSTANCE.refreshClass(DriveMeet1.class);
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }

    private ElapsedTime DisplayTimer = new ElapsedTime();
    @Override
    public void initialize_loop()
    {
        //General Info Config
        if (gamepad1.bWasPressed()) alliance = AllianceColor.RED;
        if (gamepad1.xWasPressed()) alliance = AllianceColor.BLUE;
        if (gamepad1.startWasPressed() && !gamepad1.a) odoDrive = !odoDrive;
        if (gamepad1.yWasPressed()) autoAim = !autoAim;

        //Turret Config
        if (gamepad1.dpadLeftWasPressed()) turretCmds.upOffset();
        if (gamepad1.dpadRightWasPressed()) turretCmds.downOffset();
        if (gamepad1.dpadUpWasPressed()) turretCmds.launchAngleUp();
        if (gamepad1.dpadDownWasPressed()) turretCmds.launchAngleDown();

        //Motif Config
        if (gamepad2.yWasPressed())
            motifTranslated = new DetectedColor[] {DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN};
        if (gamepad2.xWasPressed())
            motifTranslated = new DetectedColor[] {DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.GREEN};
        if (gamepad2.bWasPressed())
            motifTranslated = new DetectedColor[] {DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE};

        //Zero Config
        if (gamepad1.backWasPressed()) {
            indexerCmds.new zero().initialize();
            DisplayTimer.reset();
        }
        if (gamepad1.aWasPressed() && !gamepad1.start) {
            turretCmds.zero();
            indexerCmds.new zero(true).initialize();
            DisplayTimer.reset();
        }

        //Display Zeroed for 0.5s After Zero
        if (DisplayTimer.seconds() < 0.5) {telemetry.addLine("ZEROED"); telemetry.addLine();}

        //General Telemetry
        telemetry.addLine("General Information");
        telemetry.addData("     Alliance", alliance);
        telemetry.addData("     Use Odometry Drive", odoDrive);
        telemetry.addData("     Use Auto Aim", autoAim);
        telemetry.addData("     Motif", motifTranslated[0] + ", " + motifTranslated[1] + ", " +  motifTranslated[2]);
        telemetry.addLine("     Y = PPG, X = GPP, B = PGP");
        telemetry.addLine();

        telemetry.addLine("Indexer Data");
        telemetry.addData("     Current Position", indexerCmds.getPos());
        telemetry.addData("     Target", indexerCmds.getTarget());
        telemetry.addLine();
        telemetry.addData("     Active Slot", indexerCmds.getSlot(Slot.FIRST));
        telemetry.addData("     Top Left", indexerCmds.getSlot(Slot.THIRD));
        telemetry.addData("     Top Right", indexerCmds.getSlot(Slot.SECOND));
        telemetry.addLine();
        telemetry.addData("     Indexer P", ConstantsServo.kP);
        telemetry.addData("     Indexer I", ConstantsServo.kI);
        telemetry.addData("     Indexer D", ConstantsServo.kD);
        telemetry.addLine();

        telemetry.addLine("Turret Data");
        telemetry.addData("     Current Position", turretCmds.getPos());
        telemetry.addData("     Target", turretCmds.getTarget());
        telemetry.addData("     Offset", turretCmds.getOffset());
        telemetry.addLine();
        telemetry.addData("     Launch Angle", turretCmds.getLaunchAngle());
        telemetry.addLine();

        telemetry.addLine("Odometry Data");
        telemetry.addData("     Follower X", follower.getPose().getX());
        telemetry.addData("     Follower Y", follower.getPose().getX());
        telemetry.addData("     Follower Heading", follower.getPose().getHeading());

        telemetry.update();
    }

    private boolean started = false;

    @Override
    public void run() {
        //Run On Start Only
        if (!started) {
            DisplayTimer = null;
            driveCmds.new intakeOn().initialize();
            follower.startTeleopDrive();
            started = true;
            indexerCmds.new zero(true).initialize();
        }

        //Run next scheduled command and update subsystems
        super.run();
        follower.update();
        indexerCmds.update();
        turretCmds.update();;

        follower.setTeleOpDrive(
                gamepad1.left_stick_x * (alliance == AllianceColor.RED ? -1 : 1),
                gamepad1.left_stick_y * (alliance == AllianceColor.RED ? -1 : 1),
                -gamepad1.right_stick_x,
                !odoDrive
        );

        //Index with bob and look for empty slot
        if (!launching && indexerCmds.updateDone())
        {
            schedule(
                    new SequentialCommandGroup(
                            indexerCmds.new index(1),
                            new CommandBase() {
                                @Override public void initialize() {findUnknown();}
                                @Override public boolean isFinished() {return true;}
                            }
                    )
            );
        }

        //Handle Shooting
        if (gamepad2.yWasPressed() && indexerCmds.updateDone()) shootBasic();
        if (gamepad2.bWasPressed() && indexerCmds.updateDone()) shootMosaic();

        //Gamepad One
        if (gamepad1.backWasPressed()) {indexerCmds.new zero().initialize();}
        if (gamepad1.startWasPressed() && !gamepad1.a) odoDrive = !odoDrive;
        if (gamepad1.aWasPressed() && !gamepad1.start) autoAim = !autoAim;
        if (gamepad1.yWasPressed())
            motifTranslated = new DetectedColor[] {DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN};
        if (gamepad1.xWasPressed())
            motifTranslated = new DetectedColor[] {DetectedColor.GREEN, DetectedColor.PURPLE, DetectedColor.GREEN};
        if (gamepad1.bWasPressed())
            motifTranslated = new DetectedColor[] {DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE};

        //Gamepad Two
        if (gamepad2.dpadLeftWasPressed()) turretCmds.upOffset();
        if (gamepad2.dpadRightWasPressed()) turretCmds.downOffset();
        if (gamepad2.dpadUpWasPressed()) turretCmds.launchAngleUp();
        if (gamepad2.dpadDownWasPressed()) turretCmds.launchAngleDown();
        if (gamepad2.aWasPressed())
        {
            indexerCmds.setActive(DetectedColor.GREEN);
            findUnknown();
        }
        if (gamepad2.xWasPressed())
        {
            indexerCmds.setActive(DetectedColor.PURPLE);
            findUnknown();
        }
        if (gamepad2.leftBumperWasPressed()) indexerCmds.clearAllSlotColors();

        //General Telemetry

        Draw.drawDebug(follower);

        telemetry.addLine("General Information");
        telemetry.addData("     Alliance", alliance);
        telemetry.addData("     Use Odometry Drive", odoDrive);
        telemetry.addData("     Use Auto Aim", autoAim);
        telemetry.addData("     Motif", motifTranslated[0] + ", " + motifTranslated[1] + ", " +  motifTranslated[2]);
        telemetry.addLine("     Y = PPG, X = GPP, B = PGP");
        telemetry.addLine();

        telemetry.addLine("Indexer Data");
        telemetry.addData("     Current Position", indexerCmds.getPos());
        telemetry.addData("     Target", indexerCmds.getTarget());
        telemetry.addLine();
        telemetry.addData("     Active Slot", indexerCmds.getSlot(Slot.FIRST));
        telemetry.addData("     Top Left", indexerCmds.getSlot(Slot.THIRD));
        telemetry.addData("     Top Right", indexerCmds.getSlot(Slot.SECOND));
        telemetry.addLine();
        telemetry.addData("     Indexer P", ConstantsServo.kP);
        telemetry.addData("     Indexer I", ConstantsServo.kI);
        telemetry.addData("     Indexer D", ConstantsServo.kD);
        telemetry.addLine();

        telemetry.addLine("Turret Data");
        telemetry.addData("     Current Position", turretCmds.getPos());
        telemetry.addData("     Target", turretCmds.getTarget());
        telemetry.addData("     Offset", turretCmds.getOffset());
        telemetry.addLine();
        telemetry.addData("     Launch Angle", turretCmds.getLaunchAngle());
        telemetry.addData("     Flywheel velocity", -turretCmds.getVel());
        telemetry.addData("     Expected velocity", turretCmds.getExpectedVel());
        telemetry.addLine();

        telemetry.addLine("Odometry Data");
       telemetry.addData("Pose (mm, deg)", "x=%.1f  y=%.1f  h=%.1f",
                follower.getPose().getX(), follower.getPose().getY(), 90 - Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private  void shootBasic()
    {
        schedule(new SequentialCommandGroup(
                new CommandBase() {
                    @Override public void initialize() {launching = true;}
                    @Override public boolean isFinished() {return true;}
                },
                indexerCmds.new setSlot(Slot.FIRST),
                turretCmds.new spinUp(),
                driveCmds.new intakeOn(),
                shoot(Slot.FIRST),
                shoot(Slot.SECOND),
                shoot(Slot.THIRD),
                turretCmds.new spinDown(),
                indexerCmds.new hammerDown(),
                indexerCmds.new setSlot(Slot.FIRST),
                new CommandBase() {
                    @Override public void initialize() {launching = false;}
                    @Override public boolean isFinished() {return true;}
                }
        ));
    }

    private void shootMosaic()
    {
        schedule(new SequentialCommandGroup(
                new CommandBase() {
                    @Override public void initialize() {launching = true;}
                    @Override public boolean isFinished() {return true;}
                },
                driveCmds.new intakeOn(),
                turretCmds.new spinUp(),
                new CommandBase() {
                    private CommandBase inner;

                    @Override
                    public void initialize() {
                        Slot[] slots = pickSlotsForMotif(motifTranslated);

                        CommandBase c0 = (slots[0] != null) ? shoot(slots[0]) : new InstantCommand(() -> {});
                        CommandBase c1 = (slots[1] != null) ? shoot(slots[1]) : new InstantCommand(() -> {});
                        CommandBase c2 = (slots[2] != null) ? shoot(slots[2]) : new InstantCommand(() -> {});

                        inner = new SequentialCommandGroup(
                                c0,
                                indexerCmds.new hammerDown(),
                                c1,
                                indexerCmds.new hammerDown(),
                                c2
                        );
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
                driveCmds.new intakeOn(),
                turretCmds.new spinDown(),
                indexerCmds.new hammerDown(),
                indexerCmds.new setSlot(Slot.FIRST),
                new CommandBase() {
                    @Override public void initialize() {launching = false;}
                    @Override public boolean isFinished() {return true;}
                }
        ));
    }

    private CommandBase shoot(Slot slot) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        indexerCmds.new setSlot(slot)
                ),
                indexerCmds.new hammerUp(),
                indexerCmds.new clearFirst(),
                new CommandBase() {
                    final ElapsedTime timer = new ElapsedTime();
                    @Override public void initialize() {timer.reset();}
                    @Override public boolean isFinished() {return true;}//timer.seconds() > 2;}
                }
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

    private void findUnknown() {
            if (indexerCmds.getSlot(Slot.FIRST) == DetectedColor.UNKNOWN) indexerCmds.new setSlot(Slot.FIRST).initialize();
            else if (indexerCmds.getSlot(Slot.SECOND) == DetectedColor.UNKNOWN) indexerCmds.new setSlot(Slot.SECOND).initialize();
            else if (indexerCmds.getSlot(Slot.THIRD) == DetectedColor.UNKNOWN) indexerCmds.new setSlot(Slot.THIRD).initialize();
    }
}
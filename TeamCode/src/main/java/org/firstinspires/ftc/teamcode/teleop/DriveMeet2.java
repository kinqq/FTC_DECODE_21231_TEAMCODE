package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.seattlesolvers.solverslib.command.*;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private double power = 0.93;
    private double angle = 0.13;
    private double offset = 0;
    private int shotType = 1;
    private int lastShot = 0;
    private int shotTypeBackUp = 3;

    private final double CLOSE_POWER = 0.68;
    private final double MED_POWER = 0.93;
    private final double LONG_POWER = 0.93;
    private final double FAR_POWER = 1.0;


    private final double CLOSE_ANGLE = 0.215;
    private final double MED_ANGLE = 0.24;
    private final double LONG_ANGLE = 0.24;
    private final double FAR_ANGLE = 0.19;

    private final double CLOSE_POWER_SPEED = 0.68;
    private final double MED_POWER_SPEED = 0.93;
    private final double LONG_POWER_SPEED = 0.93;
    private final double FAR_POWER_SPEED = 1.0;

    private final double CLOSE_ANGLE_SPEED = 0.215;
    private final double MED_ANGLE_SPEED = 0.24;
    private final double LONG_ANGLE_SPEED = 0.24;
    private final double FAR_ANGLE_SPEED = 0.19;

    private int SHOT_MODES_COUNT;
    
    @Override
    public void initialize()
    {
        SHOT_MODES_COUNT = 4;

        //Subsystem Init
        indexerCmds = new MagazineCommands(hardwareMap);
        turretCmds = new TurretSubsystem(hardwareMap);
        driveCmds = new DriveCommands(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        //Indexer Init

        //Turret Init
        turretCmds.setLaunchAngle(0.85);
        turretCmds.setLaunchVel(1900);

        //Follower Init
        follower.update();

        if (GlobalState.teleOpStartPose != null) {
            follower.setStartingPose(GlobalState.teleOpStartPose);
            follower.setPose(GlobalState.teleOpStartPose);
        } else {
            follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
            follower.setPose(new Pose(72, 72, Math.toRadians(90)));
        }

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
            DisplayTimer.reset();
        }
        if (gamepad1.aWasPressed() && !gamepad1.start) {
            turretCmds.zero();
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
        telemetry.addData("     Follower Y", follower.getPose().getY());
        telemetry.addData("     Follower Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
        //drawOnlyCurrent();
    }

    private boolean started = false;
    private boolean secondStart = false;
    private final ElapsedTime startTimer = new ElapsedTime();


    @Override
    public void run() {
        //Run On Start Only
        if (!started) {
            started = true;

            DisplayTimer = null;
            //driveCmds.new intakeOn().initialize();
            follower.startTeleopDrive();
            indexerCmds.new zero(false).initialize();
            indexerCmds.new hammerDown().initialize();
            startTimer.reset();
        }
        if (startTimer.seconds() > 0.3 && !secondStart)
        {
            indexerCmds.new zero(true).initialize();

            if (GlobalState.teleOpStartPose != null) {
                follower.setStartingPose(GlobalState.teleOpStartPose);
                follower.setPose(GlobalState.teleOpStartPose);
            } else {
                follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
                follower.setPose(new Pose(72, 72, Math.toRadians(90)));
            }

            secondStart = true;
        }

        //Run next scheduled command and update subsystems
        super.run();
        follower.update();
        indexerCmds.update();
        turretCmds.update(power, angle, offset, alliance, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());

        follower.setTeleOpDrive(
                alliance == AllianceColor.RED ? -gamepad1.left_stick_y : gamepad1.left_stick_y,
                alliance == AllianceColor.RED ? -gamepad1.left_stick_x : gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );

        //Index with bob and look for empty slot
//        if (!launching && indexerCmds.updateDone() && secondStart)
//        {
//            schedule(
//                    new SequentialCommandGroup(
//                            indexerCmds.new index(),
//                            new CommandBase() {
//                                @Override public void initialize() {findUnknown();}
//                                @Override public boolean isFinished() {return true;}
//                            }
//                    )
//            );
//        }




        //Handle Power
        if (lastShot == 0)
        {
            switch (shotType)
            {
                case 1:
                    power = CLOSE_POWER;
                    angle = CLOSE_ANGLE;
                    break;
                case 2:
                    power = MED_POWER;
                    angle = MED_ANGLE;
                    break;
                case 3:
                    power = LONG_POWER;
                    angle = LONG_ANGLE;
                    break;
                case 4:
                    power = FAR_POWER;
                    angle = FAR_ANGLE;
                    break;
            }
        } else {
            switch (shotType)
            {
                case 1:
                    power = CLOSE_POWER_SPEED;
                    angle = CLOSE_ANGLE_SPEED;
                    break;
                case 2:
                    power = MED_POWER_SPEED;
                    angle = MED_ANGLE_SPEED;
                    break;
                case 3:
                    power = LONG_POWER_SPEED;
                    angle = LONG_ANGLE_SPEED;
                    break;
                case 4:
                    power = FAR_POWER_SPEED;
                    angle = FAR_ANGLE_SPEED;
                    break;
            }
        }


        //Gamepad One
        if (gamepad1.backWasPressed()) {indexerCmds.clearAllSlotColors();}
        if (gamepad1.startWasPressed() && !gamepad1.a) indexerCmds.new clearSecond().initialize();
        if (gamepad1.aWasPressed() && !gamepad1.start) indexerCmds.setActive(DetectedColor.GREEN);
        if (gamepad1.bWasPressed()) {indexerCmds.new prevSlot().initialize();}
        if (gamepad1.xWasPressed()) indexerCmds.new nextSlot().initialize();
        if (gamepad1.yWasPressed()) {}
        if (gamepad1.dpadUpWasPressed()) {}
        if (gamepad1.dpadDownWasPressed()) {}
        if (gamepad1.dpadLeftWasPressed()) {}
        if (gamepad1.dpadRightWasPressed()) {}
        if (gamepad1.leftStickButtonWasPressed()) {}
        if (gamepad1.rightStickButtonWasPressed()) {}
        if (gamepad1.leftBumperWasPressed()) indexerCmds.resetPos();
        if (gamepad1.rightBumperWasPressed()) driveCmds.new toggleIntake().initialize();

        //Gamepad Two
        if (gamepad2.backWasPressed())
        {
            schedule(new SequentialCommandGroup(
                    driveCmds.new intakeReverse(),
                    driveCmds.new intakeForward()
            ));
        }
        if (gamepad2.startWasPressed()) {}
        if (gamepad2.aWasPressed()) shotType = shotTypeBackUp;
        if (gamepad2.bWasPressed()) schedule(shootMotif());
        if (gamepad2.yWasPressed())
        {
            if (shotType == 1) shootSpeed();
            else shootBasic();
        }
        if (gamepad2.xWasPressed()) shotType = 0;
        if (gamepad2.dpadUpWasPressed())
        {
            angle += 0.005;
            angle = angle > 1 ? 1 : angle;
        }
        if (gamepad2.dpadDownWasPressed())
        {
            angle -= 0.005;
            angle = angle < 0 ? 0 : angle;

        }
        if (gamepad2.dpadLeftWasPressed()) offset += 2.5;
        if (gamepad2.dpadRightWasPressed()) offset -= 2.5;
        if (gamepad1.leftStickButtonWasPressed()) {}
        if (gamepad1.rightStickButtonWasPressed()) {}
        if (gamepad2.leftBumperWasPressed())
        {
            if (shotType == 0)
            {
                power -= 0.01;
                power = power > 1 ? 1 : power;
                power = power < 0 ? 0 : power;
            } else {
                shotType -= 1;
                shotType = shotType < 1 ? SHOT_MODES_COUNT : shotType;
                shotTypeBackUp = shotType;
            }
        }
        if (gamepad2.rightBumperWasPressed())
        {
            if (shotType == 0)
            {
                power += 0.01;
                power = power > 1 ? 1 : power;
                power = power < 0 ? 0 : power;
            } else {
                shotType += 1;
                shotType = shotType > SHOT_MODES_COUNT ? 1 : shotType;
                shotTypeBackUp = shotType;
            }
        }

        //General Telemetry

        Draw.drawDebug(follower);

        telemetry.addLine("General Information");

        telemetry.addData("     Alliance", alliance);
        telemetry.addData("     Use Odometry Drive", odoDrive);
        telemetry.addData("     Use Auto Aim", autoAim);
        telemetry.addData("     Motif", motifTranslated[0] + ", " + motifTranslated[1] + ", " +  motifTranslated[2]);
        telemetry.addLine();
        telemetry.addData("     Shot type", shotType);
        telemetry.addData("To speed", turretCmds.motorToSpeed());
        telemetry.addData("     Power", power);
        telemetry.addData("     Hood Angle", angle);
        telemetry.addData("     Flywheel velocity", turretCmds.getVel());
        telemetry.addData("     Expected velocity", turretCmds.getExpectedVel());
        telemetry.addLine();
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
        telemetry.addData("     Follower Y", follower.getPose().getY());
        telemetry.addData("     Follower Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private  void shootBasic()
    {
        lastShot = 0;
        if (shotType == 1)
        {
            power = CLOSE_POWER;
            angle = CLOSE_ANGLE;
        }
        if (shotType == 2)
        {
            power = MED_POWER;
            angle = MED_ANGLE;
        }
        if (shotType == 3)
        {
            power = LONG_POWER;
            angle = LONG_ANGLE;
        }
        if (shotType == 4)
        {
            power = FAR_POWER;
            angle = FAR_ANGLE;
        }

        schedule(new SequentialCommandGroup(
                indexerCmds.new hammerDown(),
                new ParallelCommandGroup(
                        new CommandBase() {
                            @Override public void initialize() {launching = true;}
                            @Override public boolean isFinished() {return true;}
                        },
                        indexerCmds.new setSlot(Slot.FIRST),
                        turretCmds.new spinUp(),
                        driveCmds.new intakeOn()
                ),
                indexerCmds.new clearAllSlotColors(),
                indexerCmds.new hammerUp(),
                shoot(Slot.FIRST),
                indexerCmds.new hammerDown(),
                new WaitUntilCommand(() -> turretCmds.motorToSpeed()),
                shoot(Slot.SECOND),
                indexerCmds.new hammerDown(),
                new WaitUntilCommand(() -> turretCmds.motorToSpeed()),
                shoot(Slot.THIRD),
                new ParallelCommandGroup(
                        turretCmds.new spinDown(),
                        indexerCmds.new setSlot(Slot.FIRST),
                        new CommandBase() {
                            @Override public void initialize() {launching = false;}
                            @Override public boolean isFinished() {return true;}
                        }
                ),
                indexerCmds.new hammerDown(),
                indexerCmds.new setSlot(Slot.FIRST)
        ));
    }

    private void shootSpeed()
    {
        lastShot = 1;

        if (shotType == 1)
        {
            power = CLOSE_POWER_SPEED;
            angle = CLOSE_ANGLE_SPEED;
        }
        if (shotType == 2)
        {
            power = MED_POWER_SPEED;
            angle = MED_ANGLE_SPEED;
        }
        if (shotType == 3)
        {
            power = LONG_POWER_SPEED;
            angle = LONG_ANGLE_SPEED;
        }
        if (shotType == 4)
        {
            power = FAR_POWER_SPEED;
            angle = FAR_ANGLE_SPEED;
        }

        schedule(new SequentialCommandGroup(
                indexerCmds.new hammerDown(),
                new ParallelCommandGroup(
                        new InstantCommand(() -> launching = true),
                        indexerCmds.new setSlot(Slot.FIRST),
                        turretCmds.new spinUp(),
                        driveCmds.new intakeOn()
                ),
                indexerCmds.new clearAllSlotColors(),
                indexerCmds.new hammerUp(),
                new WaitUntilCommand(() -> turretCmds.motorToSpeed()),
                indexerCmds.new setSlot(Slot.SECOND),
                new ParallelRaceGroup(
                        new WaitCommand(100),
                        new WaitUntilCommand(() -> turretCmds.motorToSpeed())
                ),
                indexerCmds.new setSlot(Slot.THIRD),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        turretCmds.new spinDown(),
                        indexerCmds.new setSlot(Slot.FIRST),
                        new InstantCommand(() -> launching = false)
                ),
                indexerCmds.new hammerDown(),
                indexerCmds.new setSlot(Slot.FIRST)
        ));
    }

    private CommandBase shootMotif()
    {
        lastShot = 0;
        if (shotType == 1)
        {
            power = CLOSE_POWER;
            angle = CLOSE_ANGLE;
        }
        if (shotType == 2)
        {
            power = MED_POWER;
            angle = MED_ANGLE;
        }
        if (shotType == 3)
        {
            power = LONG_POWER;
            angle = LONG_ANGLE;
        }
        if (shotType == 4)
        {
            power = FAR_POWER;
            angle = FAR_ANGLE;
        }

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new CommandBase() {
                            @Override public void initialize() {launching = true;}
                            @Override public boolean isFinished() {return true;}
                        },
                        driveCmds.new intakeOn(),
                        turretCmds.new spinUp(),
                        indexerCmds.new hammerDown()
                ),
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
                new ParallelCommandGroup(
                        turretCmds.new spinDown(),
                        indexerCmds.new hammerDown(),
                        indexerCmds.new setSlot(Slot.FIRST),
                        new CommandBase() {
                            @Override public void initialize() {launching = false;}
                            @Override public boolean isFinished() {return true;}
                        }
                )
        );
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
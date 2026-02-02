package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
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

    //Configurable Booleans
    private boolean autoAim = true;
    private boolean odoDrive = true;
    private boolean autoPower = true;
    private boolean index = true;
    private boolean autoDrive = false;

    //Configurable Numbers
    private double power = 0.93;
    private double angle = 0.13;
    private double offset = 0;
    private int shotType = 1;

    //Program Controlled Booleans
    private boolean launching = false;
    private boolean hasBeenFull = false;

    //Program Controlled Numbers
    private int lastShot = 0;
    private int shotTypeBackUp = 3;

    //Program Controlled Data
    public static AllianceColor alliance = AllianceColor.RED;
    private final DetectedColor[] motifTranslated = new DetectedColor[] {DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN};

    //Final Numbers
    protected final int SHOT_MODES_COUNT = 4;

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

    @Override
    public void initialize()
    {
        //Initialize Subsystems
        indexerCmds = new MagazineCommands(hardwareMap);
        turretCmds = new TurretSubsystem(hardwareMap);
        driveCmds = new DriveCommands(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        //Initialize Data From Auto
            //Follower
            follower.update();
            if (GlobalState.teleOpStartPose != null) {
                follower.setStartingPose(GlobalState.teleOpStartPose);
                follower.setPose(GlobalState.teleOpStartPose);
            } else {
                follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
                follower.setPose(new Pose(72, 72, Math.toRadians(90)));
            }

            //Alliance
            if (GlobalState.alliance != null) {
                alliance = GlobalState.alliance;
            }
            else {
                alliance = AllianceColor.RED;
            }

            //Motif
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
        PanelsConfigurables.INSTANCE.refreshClass(DriveMeet2.class);
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }

    private ElapsedTime DisplayTimer = new ElapsedTime();

    @Override
    public void initialize_loop()
    {
        //Boolean Config
            //Gamepad One
            if (gamepad1.aWasPressed()) index = true;
            if (gamepad1.bWasPressed()) index = false;
            if (gamepad1.xWasPressed()) odoDrive = true;
            if (gamepad1.yWasPressed()) odoDrive = false;
            if (gamepad1.startWasPressed())
            {
                follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
                follower.setPose(new Pose(72, 72, Math.toRadians(90)));
                DisplayTimer.reset();
            }
            if (gamepad1.rightStickButtonWasPressed()) alliance = AllianceColor.RED;
            if (gamepad1.leftStickButtonWasPressed()) alliance = AllianceColor.BLUE;

            //Gamepad Two
            if (gamepad2.aWasPressed()) autoAim = true;
            if (gamepad2.bWasPressed()) autoAim = false;
            if (gamepad2.xWasPressed()) autoPower = true;
            if (gamepad2.yWasPressed()) autoPower = false;
            if (gamepad2.startWasPressed())
            {
                turretCmds.zero();
                offset = 0;
                angle = 0;
                DisplayTimer.reset();
            }

        //Display Zeroed for 0.5s After Zero
        if (DisplayTimer.seconds() < 0.5) {telemetry.addLine("ZEROED"); telemetry.addLine();}

        //Configurables
        telemetry.addLine("Booleans");
            //Gamepad One
            telemetry.addLine("     Gamepad 1");
            telemetry.addData("     Use Color Indexing", index);
            telemetry.addData("     Use Robot Centric Movement", odoDrive);
            //Gamepad Two
            telemetry.addLine("     Gamepad 2");
            telemetry.addData("     Automatically Aim", autoAim);
            telemetry.addData("     Automatically Power Adjust", autoPower);
        telemetry.addLine();
        //Program Data
        telemetry.addLine("Program Data");
            telemetry.addData("     Current Motif", motifTranslated[0] + ", " + motifTranslated[1] + ", " +  motifTranslated[2]);
            telemetry.addData("     Current Alliance", alliance);
        telemetry.addLine();
        //Data
        telemetry.addLine("Turret Data");
            telemetry.addData("     Shot Type", shotType);
            telemetry.addData("     Expected Velocity", turretCmds.getExpectedVel());
            telemetry.addData("     Real Velocity", turretCmds.getVel());
            telemetry.addData("     Launch Angle", turretCmds.getLaunchAngle());
            telemetry.addData("     Offset", turretCmds.getOffset());
        telemetry.addLine();
        telemetry.addLine("Indexer Positional Data");
            telemetry.addData("     Target Position", indexerCmds.getTarget());
            telemetry.addData("     Current Position", indexerCmds.getPos());
            telemetry.addData("     Current Slot", indexerCmds.getActiveSlot());
        telemetry.addLine();
        telemetry.addLine("Follower Data");
            telemetry.addData("     X Position", follower.getPose().getX());
            telemetry.addData("     Y Position", follower.getPose().getX());
            telemetry.addData("     Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addLine();
        telemetry.update();

    }

    private boolean started = false;
    private boolean secondStart = false;
    private final ElapsedTime startTimer = new ElapsedTime();

    @Override
    public void run()
    {
        //Run On Start Only
        if (!started) {
            started = true;
            DisplayTimer = null;
            startTimer.reset();

            //Follower
            if (GlobalState.teleOpStartPose != null) {
                follower.setStartingPose(GlobalState.teleOpStartPose);
                follower.setPose(GlobalState.teleOpStartPose);
            } else {
                follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
                follower.setPose(new Pose(72, 72, Math.toRadians(90)));
            }
            follower.startTeleopDrive();

            //Indexer
            indexerCmds.new HammerDown().initialize();
            indexerCmds.start();
        }
        if (startTimer.milliseconds() >= 650 && !secondStart) {
            secondStart = true;
            schedule(indexerCmds.zero());
        }

        //Run next scheduled command and update subsystems
        super.run();
        follower.update();
        indexerCmds.update();
        turretCmds.update(autoPower, autoAim, power, angle, offset, alliance, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());

        if (!autoDrive)
            follower.setTeleOpDrive(
                    alliance == AllianceColor.RED ? -gamepad1.left_stick_y : gamepad1.left_stick_y,
                    alliance == AllianceColor.RED ? -gamepad1.left_stick_x : gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    !odoDrive
            );

        //Index the Magazine Either by Color or Distance
        if (index && !launching && !indexerCmds.isBusy() && secondStart)
        {
            schedule(
                    new SequentialCommandGroup(
                            indexerCmds.new Index(),
                           new InstantCommand(this::findEmpty)
                    )
            );
        } else if (!launching && !indexerCmds.isBusy() && secondStart) {
            schedule(
                    new SequentialCommandGroup(
                            indexerCmds.new DistanceIndex(),
                            new InstantCommand(this::findEmpty)
                    )
            );
        }
//        if (indexerCmds.isFull() && !hasBeenFull)
//        {
//            hasBeenFull = true;
//            driveCmds.new intakeReverse().initialize();
//        } else if (!indexerCmds.isFull() && hasBeenFull) {
//            hasBeenFull = false;
//            driveCmds.new intakeOn().initialize();
//        }

        //Update Controllers
        controls();

        //Handle Shot Types
        if (autoPower) shotType = -1;
        else if (lastShot != 0)
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

        //Draw Follower
        Draw.drawDebug(follower);

        //Telemetry
            //Quick Info
            telemetry.addData("G", indexerCmds.getAnalog());

            telemetry.addLine("Quick Info (Important During Comp)");
                telemetry.addData("     Is Motor at Expected Vel", turretCmds.motorToSpeed());
                telemetry.addData("     Is Indexer at Expected Position", !indexerCmds.isBusy());
                telemetry.addData("     Shot Type", shotType);
                telemetry.addData("     Flywheel Power", power);
                telemetry.addData("     Hood Angle", angle);
                telemetry.addData("     Expected Velocity", turretCmds.getExpectedVel());
                telemetry.addData("     Actual Velocity", turretCmds.getVel());
                telemetry.addData("     Distance From Goal", turretCmds.getDist());
                telemetry.addData("     Active Indexer Slot", indexerCmds.getActiveSlot());
                //telemetry.addData("     Attempting to Auto Drive", autoDrive);
            telemetry.addLine();
            telemetry.addLine("General Information");
                telemetry.addData("     Motif", motifTranslated[0] + ", " + motifTranslated[1] + ", " +  motifTranslated[2]);
                telemetry.addData("     Alliance", alliance);
                telemetry.addData("     Auto Aiming", autoAim);
                telemetry.addData("     Field Centric", odoDrive);
                telemetry.addData("     Auto Power", autoPower);
            telemetry.addLine();


        telemetry.addLine("Indexer Data");
        telemetry.addData("     Current Position", indexerCmds.getPos());
        telemetry.addData("     Current Position (real)", indexerCmds.realServoPos());
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

    private void controls()
    {
        //Gamepad One
        if (gamepad1.backWasPressed()) indexerCmds.new clearAllSlotColors().initialize();
        if (gamepad1.startWasPressed() && !gamepad1.a)
        {
            follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
            follower.setPose(new Pose(72, 72, Math.toRadians(90)));
        }
        if (gamepad1.aWasPressed() && !gamepad1.start) indexerCmds.setActive(DetectedColor.GREEN);
        if (gamepad1.bWasPressed())
        {
            schedule(indexerCmds.prevSlot());
        }

        if (gamepad1.xWasPressed())
        {
            schedule(indexerCmds.nextSlot());
        }
        if (gamepad1.yWasPressed()) indexerCmds.setActive(DetectedColor.PURPLE);
        if (gamepad1.rightBumperWasPressed()) driveCmds.new toggleIntake().initialize();
        if (gamepad1.leftBumperWasPressed()) driveCmds.new intakeForward().initialize();
        if (gamepad1.rightStickButtonWasPressed()) index = !index;
        if (gamepad1.leftStickButtonWasPressed()) odoDrive = !odoDrive;
        if (gamepad1.dpadUpWasPressed())
        {
            autoDrive = true;

            Pose start = new Pose(follower.getPose().getX(), follower.getPose().getY());
            Pose end = new Pose(38.5, 33.5);
            PathChain base = follower
                    .pathBuilder()
                    .addPath(new BezierLine(start, end))
                    .setLinearHeadingInterpolation(
                            follower.getHeading(),
                            Math.toRadians(0)
                    )
                    .build();
            follower.followPath(base);
        }
        else if (gamepad1.dpadUpWasReleased())
        {
            autoDrive = false;
            follower.startTeleopDrive();
        }

        //Gamepad Two
        if (gamepad2.backWasPressed()) driveCmds.new intakeReverse().initialize();
        if (gamepad2.backWasReleased()) driveCmds.new intakeForward().initialize();
        if (gamepad2.aWasPressed()) shotType = shotTypeBackUp;
        if (gamepad2.bWasPressed()) schedule(shootMotif());
        if (gamepad2.yWasPressed())
        {
            if (shotType == 1 || turretCmds.getDist() < 75) shootSpeed();
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
        if (gamepad2.leftStickButtonWasPressed()) autoAim = !autoAim;
        if (gamepad2.rightStickButtonWasPressed())
        {
            shotType = 1;
            autoPower = !autoPower;
        }
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
    }


    private void shootBasic()
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
                indexerCmds.new HammerDown(),
                new ParallelCommandGroup(
                        new CommandBase() {
                            @Override public void initialize() {launching = true;}
                            @Override public boolean isFinished() {return true;}
                        },
                        indexerCmds.new SetSlot(Slot.FIRST),
                        turretCmds.new spinUp(),
                        driveCmds.new intakeOn()
                ),
                indexerCmds.new clearAllSlotColors(),
                indexerCmds.new HammerUp(),
                shoot(Slot.FIRST),
                indexerCmds.new HammerDown(),
                new WaitUntilCommand(() -> turretCmds.motorToSpeed()),
                shoot(Slot.SECOND),
                indexerCmds.new HammerDown(),
                new WaitUntilCommand(() -> turretCmds.motorToSpeed()),
                shoot(Slot.THIRD),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        turretCmds.new spinDown(),
                        indexerCmds.new SetSlot(Slot.FIRST),
                        new CommandBase() {
                            @Override public void initialize() {launching = false;}
                            @Override public boolean isFinished() {return true;}
                        }
                ),
                indexerCmds.new HammerDown(),
                indexerCmds.new SetSlot(Slot.FIRST)
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
                indexerCmds.new HammerDown(),
                new ParallelCommandGroup(
                        new InstantCommand(() -> launching = true),
                        indexerCmds.new SetSlot(Slot.FIRST),
                        turretCmds.new spinUp(),
                        driveCmds.new intakeOn()
                ),
                indexerCmds.new clearAllSlotColors(),
                indexerCmds.new HammerUp(),
                new WaitUntilCommand(() -> turretCmds.motorToSpeed()),
                indexerCmds.new SetSlot(Slot.SECOND),
                new ParallelCommandGroup(
                        new WaitCommand(100),
                        new WaitUntilCommand(() -> turretCmds.motorToSpeed())
                ),
                indexerCmds.new SetSlot(Slot.THIRD),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        turretCmds.new spinDown(),
                        indexerCmds.new SetSlot(Slot.FIRST),
                        new InstantCommand(() -> launching = false)
                ),
                indexerCmds.new HammerDown(),
                indexerCmds.new SetSlot(Slot.FIRST)
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
                        indexerCmds.new HammerDown()
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
                                //indexerCmds.new HammerDown(),
                                c1,
                                //indexerCmds.new HammerDown(),
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
                new WaitCommand(500),
                new ParallelCommandGroup(
                        turretCmds.new spinDown(),
                        indexerCmds.new HammerDown(),
                        indexerCmds.new SetSlot(Slot.FIRST),
                        indexerCmds.new clearAllSlotColors(),
                        new CommandBase() {
                            @Override public void initialize() {launching = false;}
                            @Override public boolean isFinished() {return true;}
                        }
                )
        );
    }

    private CommandBase shoot(Slot slot) {
        return new SequentialCommandGroup(
                indexerCmds.setSlot(slot),
                indexerCmds.hammerUp(),
                indexerCmds.setColor(Slot.FIRST, DetectedColor.UNKNOWN)
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

    private void findEmpty() {
            if (indexerCmds.getSlot(Slot.FIRST) == DetectedColor.EMPTY) indexerCmds.new SetSlot(Slot.FIRST).initialize();
            else if (indexerCmds.getSlot(Slot.SECOND) == DetectedColor.EMPTY) indexerCmds.new SetSlot(Slot.SECOND).initialize();
            else if (indexerCmds.getSlot(Slot.THIRD) == DetectedColor.EMPTY) indexerCmds.new SetSlot(Slot.THIRD).initialize();
    }

}
package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.constant.AllianceColor;
import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.constant.Slot;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.Draw;
import org.firstinspires.ftc.teamcode.subsystem.commands.IndexerCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.util.GlobalState;
import org.firstinspires.ftc.teamcode.util.MotifUtil;

@TeleOp (name = "Drive State")
public class DriveSTATE extends CommandOpMode
{
    private IndexerCommands indexerCmds;
    private TurretCommands turretCmds;
    private IntakeCommands intakeCmds;
    private Follower follower;

    private Servo light;
    private ElapsedTime lightTimer;

    private boolean autoPower;
    private boolean autoAim;
    private double velocity;
    private double angle;
    private double offset;
    private double idlePower;

    private boolean robotCentric;

    private AllianceColor alliance;
    private LimelightCommands.Motif motif;
    private DetectedColor[] motifTranslated;

    private boolean shooting;
    private boolean humanPlayer;

    @Override
    public void initialize()
    {
        indexerCmds = new IndexerCommands(hardwareMap);
        turretCmds = new TurretCommands(hardwareMap);
        intakeCmds = new IntakeCommands(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        light = hardwareMap.get(Servo.class, "light");
        light.setDirection(Servo.Direction.REVERSE);

        lightTimer = new ElapsedTime();
        lightTimer.reset();

        turretCmds.zero();

        if (GlobalState.teleOpStartPose != null)
        {
            follower.setStartingPose(GlobalState.teleOpStartPose);
        } else
        {
            follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        }
        follower.update();

        if (GlobalState.alliance != null)
        {
            alliance = GlobalState.alliance;
        } else
        {
            alliance = AllianceColor.RED;
        }

        if (GlobalState.lastMotif != LimelightCommands.Motif.UNKNOWN)
        {
            motif = GlobalState.lastMotif;
            motifTranslated = MotifUtil.motifToColors(motif);
        } else
        {
            motifTranslated = new DetectedColor[] {DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE};
        }

        autoPower = false;
        autoAim = true;
        velocity = 1340;
        angle = 0.59;
        offset = 0;
        idlePower = 0;

        robotCentric = false;

        shooting = false;
        humanPlayer = false;

        //Enable Panels Telemetry
        PanelsConfigurables.INSTANCE.refreshClass(DriveSTATE.class);
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }

    private double lightPos = 0.3;
    private boolean lightForward = true;

    @Override
    public void initialize_loop()
    {
        if (lightPos > 0.69) lightForward = false;
        if (lightPos < 0.3) lightForward = true;
        if (lightTimer.milliseconds() > 25)
        {
            lightPos = lightForward ? lightPos + 0.01 : lightPos - 0.01;
            lightTimer.reset();
        }
        light.setPosition(lightPos);
    }

    private boolean started = false;

    @Override
    public void run()
    {
        if (!started)
        {
            turretCmds.start();
            intakeCmds.start();
            indexerCmds.start();

            follower.startTeleopDrive();
            follower.update();

            started = true;
        }

        super.run();
        follower.update();
        indexerCmds.update();
        turretCmds.update(autoPower, autoAim, velocity, angle, offset, idlePower, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), alliance);
        //light.setPosition(lightColor);

        follower.setTeleOpDrive(
                alliance == AllianceColor.RED ? -gamepad1.left_stick_y : gamepad1.left_stick_y,
                alliance == AllianceColor.RED ? -gamepad1.left_stick_x : gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                robotCentric
        );

        if (!shooting && !indexerCmds.indexerIsMoving())
        {
            if (lightTimer.seconds() < 0.25 && humanPlayer)
                light.setPosition(0.58);
            else if (lightTimer.seconds() < 0.5 && indexerCmds.isFull())
                light.setPosition(0.48);
            if (lightTimer.seconds() > 0.5)
            {
                if (indexerCmds.getIntakeSlotColor() == DetectedColor.GREEN) light.setPosition(0.51);
                else if (indexerCmds.getIntakeSlotColor() == DetectedColor.PURPLE) light.setPosition(0.32);
                else light.setPosition(0);
            } if (lightTimer.seconds() > 1) lightTimer.reset();

            indexerCmds.getDetectedColor();
        }

        controls();

        Draw.drawDebug(follower);
        telemetry.addData("Contents", indexerCmds.getContents());
        telemetry.addData("Is Full", indexerCmds.isFull());
        telemetry.addData("Distance From Goal", turretCmds.getDistToGoal());
        telemetry.addData("Velocity", turretCmds.getExpectedVelocity());
        telemetry.addData("Angle", turretCmds.getHoodAngle());
        telemetry.addData("Real Vel", turretCmds.getRealVelocity());
        telemetry.addData("Motor to Speed", turretCmds.flywheelAtExpectedSpeed());
        telemetry.addData("Indexer Busy", indexerCmds.indexerIsMoving());
        telemetry.addData("First Rev", indexerCmds.isOnFirstRev());
        telemetry.addData("Active Slot", indexerCmds.getIntakeSlot());
        telemetry.update();
    }

    private void controls()
    {
        if (gamepad1.backWasPressed()) indexerCmds.clearContents();
        if (gamepad1.startWasPressed()) follower.setPose(new Pose(72, 144, Math.toRadians(90)));
        if (gamepad1.aWasPressed())
        {
            autoPower = false;
            humanPlayer = true;
            idlePower = -0.2;
            angle = 0.19;
            intakeCmds.intakeReverse();
            intakeCmds.intakeOn();
            intakeCmds.hammerActive();
        }
        if (gamepad1.aWasReleased())
        {
            autoPower = true;
            humanPlayer = false;
            idlePower = 0;
            intakeCmds.intakeForward();
            intakeCmds.intakeOn();
            intakeCmds.hammerPassive();
        }
        if (gamepad1.bWasPressed()) indexerCmds.prevSlot();
        if (gamepad1.xWasPressed()) indexerCmds.nextSlot();
        if (gamepad1.leftBumperWasPressed())
        {
            intakeCmds.intakeReverse();
            intakeCmds.intakeOn();
            intakeCmds.hammerActive();
        }
        if (gamepad1.leftBumperWasReleased())
        {
            intakeCmds.intakeForward();
            intakeCmds.intakeOn();
            intakeCmds.hammerPassive();
        }
        if (gamepad1.rightBumperWasPressed()) intakeCmds.toggleIntake();
        if (gamepad1.leftStickButtonWasPressed())
        {
            if (alliance == AllianceColor.RED) alliance = AllianceColor.BLUE;
            else if (alliance == AllianceColor.BLUE) alliance = AllianceColor.RED;
        }
        if (gamepad1.rightStickButtonWasPressed()) robotCentric = !robotCentric;


//        if (gamepad2.backWasPressed())
//        {
//            intakeCmds.intakeReverse();
//            intakeCmds.intakeOn();
//        }
//        if (gamepad2.backWasReleased())
//        {
//            intakeCmds.intakeForward();
//            intakeCmds.intakeOn();
//        }
//        if (gamepad2.aWasPressed()) schedule(shootColor(DetectedColor.GREEN));
//        if (gamepad2.bWasPressed()) schedule(shootMotif());
//        if (gamepad2.xWasPressed()) schedule(shootColor(DetectedColor.PURPLE));
//        if (gamepad2.yWasPressed())
//        {
//            //if (turretCmds.getExpectedVelocity() > 1500)
//            //    schedule(shootSlow());
//            //else schedule(shootRapid());
//            schedule(shootRapid());
//        }
//        if (gamepad2.dpadLeftWasPressed()) offset += 2.5;
//        if (gamepad2.dpadRightWasPressed()) offset -= 2.5;
//        if (gamepad2.dpadUpWasPressed()) angle += 0.01;
//        if (gamepad2.dpadDownWasPressed()) angle -= 0.01;
//        if (gamepad2.leftBumperWasPressed()) schedule(shootColor(indexerCmds.getIntakeSlotColor()));
//        if (gamepad2.rightBumperWasPressed()) turretCmds.spinUpToVelocity();
//        if (gamepad2.leftStickButtonWasPressed()) autoAim = !autoAim;

        if (gamepad2.leftBumperWasPressed()) velocity -= 10;
        if (gamepad2.rightBumperWasPressed()) velocity += 10;
        if (gamepad2.dpadUpWasPressed()) angle += 0.01;
        if (gamepad2.dpadDownWasPressed()) angle -= 0.01;
        if (gamepad2.yWasPressed()) schedule(shootRapid());
        if (gamepad2.dpadLeftWasPressed()) offset += 1;
        if (gamepad2.dpadRightWasPressed()) offset -= 1;
        if (gamepad2.startWasPressed()) turretCmds.deactivateLauncher();
    }

    private CommandBase shootRapid()
    {
        boolean moveForward = false;
        if (indexerCmds.isOnFirstRev())
        {
            if (indexerCmds.getIntakeSlot() == Slot.FIRST) moveForward = true;
            if (indexerCmds.getIntakeSlot() == Slot.SECOND) moveForward = true;
        }

        shooting = true;
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        intakeCmds.new HammerPassive(),
                        intakeCmds.new IntakeOn(),
                        turretCmds.new SpinUp(),
                        new SetLight(0.4)
                ),
                new WaitUntilCommand(() -> turretCmds.flywheelAtExpectedSpeed()),
                //new SetLight(0.69),
                intakeCmds.new HammerActive(),
                new WaitCommand(150),
                !moveForward ? indexerCmds.new PrevSlot() : indexerCmds.new NextSlot(),
                new WaitCommand(200),
                !moveForward ? indexerCmds.new PrevSlot() : indexerCmds.new NextSlot(),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        indexerCmds.new ClearContents(),
                        indexerCmds.new SetSlot(Slot.FIRST),
                        intakeCmds.new HammerPassive(),
                        turretCmds.new SpinDown(),
                        new InstantCommand(() -> shooting = false)
                )
        );
    }

    private CommandBase shootMotif()
    {
        Slot first;
        Slot upOne;
        Slot downOne;
        boolean forward = false;

        if (indexerCmds.getSlotColor(Slot.FIRST) == motifTranslated[0])
            first = Slot.FIRST;
        else if (indexerCmds.getSlotColor(Slot.SECOND) == motifTranslated[0])
            first = Slot.SECOND;
        else if (indexerCmds.getSlotColor(Slot.THIRD) == motifTranslated[0])
            first = Slot.THIRD;
        else first = Slot.FIRST;

        if (first == Slot.FIRST)
        {
            upOne = Slot.SECOND;
            downOne = Slot.THIRD;
        }
        else if (first == Slot.SECOND)
        {
            upOne = Slot.THIRD;
            downOne = Slot.FIRST;
        }
        else
        {
            upOne = Slot.FIRST;
            downOne= Slot.SECOND;
        }

        if (indexerCmds.getSlotColor(upOne) == motifTranslated[1]) forward = true;
        else forward = indexerCmds.getSlotColor(downOne) != motifTranslated[1];

        if (
                (forward && (first == Slot.SECOND || first == Slot.THIRD) && !indexerCmds.isOnFirstRev())
                ||
                (!forward && (first == Slot.FIRST || first == Slot.SECOND) && indexerCmds.isOnFirstRev())
        ) {
            if (indexerCmds.getSlotColor(first) == DetectedColor.GREEN) forward = !forward;
            else
            {
                if (indexerCmds.getSlotColor(Slot.FIRST) == motifTranslated[0] && first != Slot.FIRST)
                    first = Slot.FIRST;
                else if (indexerCmds.getSlotColor(Slot.SECOND) == motifTranslated[0] && first != Slot.SECOND)
                    first = Slot.SECOND;
                else if (indexerCmds.getSlotColor(Slot.THIRD) == motifTranslated[0] && first != Slot.THIRD)
                    first = Slot.THIRD;
                else first = Slot.SECOND;

                if (first == Slot.FIRST)
                {
                    upOne = Slot.SECOND;
                    downOne = Slot.THIRD;
                }
                else if (first == Slot.SECOND)
                {
                    upOne = Slot.THIRD;
                    downOne = Slot.FIRST;
                }
                else
                {
                    upOne = Slot.FIRST;
                    downOne= Slot.SECOND;
                }

                if (indexerCmds.getSlotColor(upOne) == motifTranslated[1]) forward = true;
                else forward = indexerCmds.getSlotColor(upOne) != motifTranslated[1];
            }
        }

        shooting = true;

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        indexerCmds.new SetSlot(first),
                        intakeCmds.new HammerPassive(),
                        intakeCmds.new IntakeOn(),
                        turretCmds.new SpinUp(),
                        new SetLight(0.4)
                ),
                new WaitUntilCommand(() -> turretCmds.flywheelAtExpectedSpeed()),
                intakeCmds.new HammerActive(),
                new WaitCommand(200),
                !forward ? indexerCmds.new PrevSlot() : indexerCmds.new NextSlot(),
                new WaitCommand(200),
                !forward ? indexerCmds.new PrevSlot() : indexerCmds.new NextSlot(),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        indexerCmds.new ClearContents(),
                        indexerCmds.new SetSlot(Slot.FIRST),
                        intakeCmds.new HammerPassive(),
                        turretCmds.new SpinDown(),
                        new InstantCommand(() -> shooting = false)
                )
        );
    }

    private CommandBase shootColor(DetectedColor color)
    {
        Slot slot;
        if (indexerCmds.getIntakeSlotColor() == color) slot = indexerCmds.getIntakeSlot();
        else if (indexerCmds.getSlotColor(Slot.FIRST) == color) slot = Slot.FIRST;
        else if (indexerCmds.getSlotColor(Slot.SECOND) == color) slot = Slot.SECOND;
        else if (indexerCmds.getSlotColor(Slot.THIRD) == color) slot = Slot.THIRD;
        else return new InstantCommand();

        shooting = true;
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        indexerCmds.new SetSlot(slot),
                        intakeCmds.new HammerPassive(),
                        intakeCmds.new IntakeOn(),
                        turretCmds.new SpinUp(),
                        new SetLight(0.4)
                ),
                intakeCmds.new HammerActive(),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        intakeCmds.new HammerPassive(),
                        turretCmds.new SpinDown(),
                        new InstantCommand(() -> shooting = false)
                )
        );
    }

    private class SetLight extends CommandBase
    {
        double color;

        public SetLight(double color) {this.color = color;}

        @Override
        public void initialize() {light.setPosition(color);}

        @Override
        public boolean isFinished() {return true;}
    }

}

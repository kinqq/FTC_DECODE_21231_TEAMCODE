package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.constant.AllianceColor;
import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.constant.Motif;
import org.firstinspires.ftc.teamcode.constant.Slot;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.Draw;
import org.firstinspires.ftc.teamcode.subsystem.commands.IndexerCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.LaunchCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.PTOCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.util.GlobalState;
import org.firstinspires.ftc.teamcode.util.MotifUtil;

@TeleOp (name = "Drive State")
public class DriveSTATE extends CommandOpMode
{
    private IndexerCommands indexerCmds;
    private TurretCommands turretCmds;
    private IntakeCommands intakeCmds;
    private LaunchCommands launchCmds;
    private PTOCommands ptoCmds;
    private Follower follower;

    private ElapsedTime lightTimer;

    private boolean autoPower;
    private boolean autoAim;
    private double velocity;
    private double angle;
    private double offset;
    private double idlePower;

    private boolean robotCentric;
    private double speedMultiplier;

    private AllianceColor alliance;
    private Motif motif;
    private DetectedColor[] motifTranslated;

    private boolean humanPlayer;
    private boolean basing;
    private boolean basingBegun;

    @Override
    public void initialize()
    {
        indexerCmds = new IndexerCommands(hardwareMap);
        turretCmds = new TurretCommands(hardwareMap);
        intakeCmds = new IntakeCommands(hardwareMap);
        launchCmds = new LaunchCommands(
                hardwareMap,
                indexerCmds,
                turretCmds,
                intakeCmds
        );
        ptoCmds = new PTOCommands(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

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

        if (GlobalState.lastMotif != Motif.UNKNOWN)
        {
            LimelightCommands.Motif motif = GlobalState.lastMotif;
            motifTranslated = MotifUtil.motifToColors(motif);
        } else
        {
            motifTranslated = new DetectedColor[] {DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE};
        }

        speedMultiplier = 1;

        autoPower = false;
        autoAim = true;
        velocity = 1340;
        angle = 0.59;
        offset = 0;
        idlePower = 0;

        robotCentric = false;

        humanPlayer = false;
        basing = false;
        basingBegun = false;

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
        launchCmds.setLight(lightPos);
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

        if (!basing) mainLoop();
        else if (!basingBegun)
        {
            schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> launchCmds.killAll()),
                        new InstantCommand(() -> indexerCmds.killPower()),
                        new InstantCommand(() -> turretCmds.killPower()),
                        new InstantCommand(() -> intakeCmds.killPower()),
                        launchCmds.new SetLight(0.72)
                ),
                ptoCmds.new EngageClutch(),
                new ParallelCommandGroup(
                        ptoCmds.new ThrottleFront(),
                        ptoCmds.new ThrottleBack()
                ),
                ptoCmds.new KillFront(),
                new WaitCommand(5000),
                ptoCmds.new IdleBack()
            ));
            basingBegun = true;
        }
    }

    private void mainLoop()
    {

        follower.update();
        indexerCmds.update();
        turretCmds.update(autoPower, autoAim, velocity, angle, offset, idlePower, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), alliance);
        //light.setPosition(lightColor);

        follower.setTeleOpDrive(
                alliance == AllianceColor.RED ? -gamepad1.left_stick_y * speedMultiplier : gamepad1.left_stick_y * speedMultiplier,
                alliance == AllianceColor.RED ? -gamepad1.left_stick_x * speedMultiplier : gamepad1.left_stick_x * speedMultiplier,
                -gamepad1.right_stick_x * speedMultiplier,
                robotCentric
        );

        if (!launchCmds.isShooting() && !indexerCmds.indexerIsMoving())
        {
            if (lightTimer.seconds() < 0.25 && humanPlayer)
                launchCmds.setLight(0.58);
            else if (lightTimer.seconds() < 0.5 && indexerCmds.isFull())
                launchCmds.setLight(0.48);
            if (lightTimer.seconds() > 0.5)
            {
                if (indexerCmds.getIntakeSlotColor() == DetectedColor.GREEN) launchCmds.setLight(0.51);
                else if (indexerCmds.getIntakeSlotColor() == DetectedColor.PURPLE) launchCmds.setLight(0.32);
                else launchCmds.setLight(0);
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
        if (gamepad1.left_trigger > 0.5) speedMultiplier = 0.5;
        else speedMultiplier = 1;
        if (gamepad1.leftStickButtonWasPressed())
        {
            if (alliance == AllianceColor.RED) alliance = AllianceColor.BLUE;
            else if (alliance == AllianceColor.BLUE) alliance = AllianceColor.RED;
        }
        if (gamepad1.rightStickButtonWasPressed()) robotCentric = !robotCentric;


        if (gamepad2.backWasPressed())
        {
            intakeCmds.intakeReverse();
            intakeCmds.intakeOn();
        }
        if (gamepad2.backWasReleased())
        {
            intakeCmds.intakeForward();
            intakeCmds.intakeOn();
        }
        if (gamepad2.aWasPressed()) schedule(launchCmds.shootColor(gamepad2, DetectedColor.GREEN));
        if (gamepad2.bWasPressed()) schedule(launchCmds.shootMotif(gamepad2, motifTranslated));
        if (gamepad2.xWasPressed()) schedule(launchCmds.shootColor(gamepad2, DetectedColor.PURPLE));
        if (gamepad2.yWasPressed()) schedule(launchCmds.shootRapid(gamepad2));
        if (gamepad2.dpadLeftWasPressed()) offset += 2.5;
        if (gamepad2.dpadRightWasPressed()) offset -= 2.5;
        if (gamepad2.dpadUpWasPressed()) angle += 0.01;
        if (gamepad2.dpadDownWasPressed()) angle -= 0.01;
        if (gamepad2.leftBumperWasPressed()) schedule(launchCmds.shootColor(gamepad2, indexerCmds.getIntakeSlotColor()));
        if (gamepad2.rightBumperWasPressed()) turretCmds.spinUpToVelocity();
        if (gamepad2.leftStickButtonWasPressed()) autoAim = !autoAim;

//        if (gamepad2.leftBumperWasPressed()) velocity -= 10;
//        if (gamepad2.rightBumperWasPressed()) velocity += 10;
//        if (gamepad2.dpadUpWasPressed()) angle += 0.01;
//        if (gamepad2.dpadDownWasPressed()) angle -= 0.01;
//        if (gamepad2.yWasPressed()) schedule(launchCmds.shootRapid());
//        if (gamepad2.dpadLeftWasPressed()) offset += 1;
//        if (gamepad2.dpadRightWasPressed()) offset -= 1;
//        if (gamepad2.startWasPressed()) turretCmds.deactivateLauncher();
    }





}

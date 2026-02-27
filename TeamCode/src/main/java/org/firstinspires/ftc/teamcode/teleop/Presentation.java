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
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.constant.AllianceColor;
import org.firstinspires.ftc.teamcode.constant.DetectedColor;
import org.firstinspires.ftc.teamcode.constant.Motif;
import org.firstinspires.ftc.teamcode.pedropathing.PedroConstants;
import org.firstinspires.ftc.teamcode.pedropathing.Draw;
import org.firstinspires.ftc.teamcode.subsystem.commands.IndexerCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.LaunchCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.PTOCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.util.GlobalState;
import org.firstinspires.ftc.teamcode.util.MotifUtil;

@TeleOp (name = "PRESENTATION")
public class Presentation extends CommandOpMode
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

        follower = PedroConstants.createFollower(hardwareMap);

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
            Motif motif = GlobalState.lastMotif;
            motifTranslated = MotifUtil.motifToColors(motif);
        } else
        {
            motifTranslated = new DetectedColor[] {DetectedColor.PURPLE, DetectedColor.GREEN, DetectedColor.PURPLE};
        }

        speedMultiplier = 1;

        autoPower = false;
        autoAim = true;
        velocity = 500;
        angle = 0.5;
        offset = 0;
        idlePower = 0;

        robotCentric = false;

        humanPlayer = false;
        basing = false;
        basingBegun = false;

        //Enable Panels Telemetry
        PanelsConfigurables.INSTANCE.refreshClass(Presentation.class);
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

            follower.update();

            ptoCmds.new DisengageClutch().initialize();

            started = true;
        }
        super.run();

        if (!basing) mainLoop();
        else if (!basingBegun)
        {
            schedule(new ParallelCommandGroup(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> turretCmds.update(false,false, 0, 0, 180, 0, 0, 0, 0, AllianceColor.BLUE)),
                            new WaitCommand(2000),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> launchCmds.killAll()),
                                    new InstantCommand(() -> indexerCmds.killPower()),
                                    new InstantCommand(() -> turretCmds.killPower()),
                                    new InstantCommand(() -> intakeCmds.killPower())
                            ),
                            ptoCmds.new EngageClutch(),
                            new ParallelCommandGroup(
                                    ptoCmds.new ThrottleFront(),
                                    ptoCmds.new ThrottleBack()
                            ),
                            ptoCmds.new KillFront()
                    ),
                    new CommandBase()
                    {
                            final ElapsedTime timer = new ElapsedTime();
                            @Override
                            public void initialize() {timer.reset();}
                            @Override
                            public void execute()
                            {
                                if (timer.milliseconds() < 100) launchCmds.setLight(0.72);
                                if (timer.milliseconds() > 100) launchCmds.setLight(0.9);
                                if (timer.milliseconds() > 200) timer.reset();

                                if (gamepad1.aWasPressed())
                                {
                                    ptoCmds.new ThrottleBack().initialize();
                                    ptoCmds.new KillFront().initialize();
                                }
                                if (gamepad1.guideWasPressed()) terminateOpModeNow();

                                telemetry.addLine("RAISING | PRESS CENTER BUTTON TO KILL OP MODE");
                                telemetry.update();
                            }
                    }
            ));
            basingBegun = true;
        }
    }

    private void mainLoop()
    {
        follower.update();
        indexerCmds.update();
        turretCmds.update(autoPower, autoAim, velocity, angle, offset, idlePower, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), alliance);

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
        if (gamepad1.aWasPressed())
        {
            autoPower = false;
            launchCmds.shootRapid(gamepad2);
        }
        if (gamepad1.aWasReleased())
        {
            launchCmds.killAll();
        }
        if (gamepad1.bWasPressed()) autoPower = true;
        if (gamepad1.startWasPressed())
        {
            autoPower = false;
            humanPlayer = true;
            idlePower = -0.4;
            angle = 0.19;
            intakeCmds.intakeReverse();
            intakeCmds.intakeOn();
            intakeCmds.hammerActive();
            turretCmds.deactivateLauncher();
        }
        if (gamepad1.startWasReleased())
        {
            humanPlayer = false;
            idlePower = 0;
            intakeCmds.intakeForward();
            intakeCmds.intakeOff();
            intakeCmds.hammerPassive();
        }
        if (gamepad1.leftBumperWasPressed()) indexerCmds.prevSlot();
        if (gamepad1.rightBumperWasPressed()) indexerCmds.nextSlot();
        if (gamepad1.backWasPressed())
        {
            intakeCmds.intakeForward();
            intakeCmds.intakeOn();
            intakeCmds.hammerActive();
        }
        if (gamepad1.backWasReleased())
        {
            intakeCmds.intakeOff();
            intakeCmds.hammerPassive();
        }
        if (gamepad1.xWasPressed()) intakeCmds.intakeOn();
        if (gamepad1.xWasReleased()) intakeCmds.intakeOff();
        if (gamepad1.yWasPressed())
        {
            autoPower = false;
            turretCmds.deactivateLauncher();
            idlePower = 1;
        }
        if (gamepad1.yWasReleased()) idlePower = 0;
    }





}

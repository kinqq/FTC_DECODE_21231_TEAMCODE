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
    private ElapsedTime gameTimer;

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

    private boolean hasBeenFull;
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

        gameTimer = new ElapsedTime();
        gameTimer.reset();

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

        autoPower = true;
        autoAim = true;
        velocity = 1340;
        angle = 0.59;
        offset = 0;
        idlePower = 0;

        robotCentric = false;

        hasBeenFull = false;
        humanPlayer = false;
        basing = false;
        basingBegun = false;

        //Enable Panels Telemetry
        PanelsConfigurables.INSTANCE.refreshClass(DriveSTATE.class);
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        telemetry.speak("Suga Pie is READY TO GO!!!");
        telemetry.update();
    }

    private double lightPos = 0.3;
    private boolean lightForward = true;
    private boolean endGame = false;
    private boolean tenS = false;
    private boolean fiveS = false;

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

            ptoCmds.new DisengageClutch().initialize();

            gameTimer.reset();

            started = true;
            telemetry.speak("Finally, yall suck");
            telemetry.update();
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
                            ptoCmds.new VelLift()
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

        if (gameTimer.seconds() >= 100 && !endGame)
        {
            gamepad1.rumble(1000);
            endGame = true;
        }
        if (gameTimer.seconds() >= 110 && !tenS)
        {
            gamepad1.rumble(1000);
            tenS = true;
        }
        if (gameTimer.seconds() >= 115 && !fiveS)
        {
            gamepad1.rumble(5000);
            fiveS = true;
        }
    }

    private void mainLoop()
    {
        follower.update();
        indexerCmds.update();
        turretCmds.update(autoPower, autoAim, velocity, angle, offset, idlePower, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), alliance);
        //turretCmds.updateWithPhysics(autoPower, autoAim, velocity, angle, offset, idlePower, follower, alliance);

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
            {
                launchCmds.setLight(0.48);
                gamepad2.rumble(50);
            }
            if (lightTimer.seconds() > 0.5)
            {
                if (indexerCmds.getIntakeSlotColor() == DetectedColor.GREEN) launchCmds.setLight(0.51);
                else if (indexerCmds.getIntakeSlotColor() == DetectedColor.PURPLE) launchCmds.setLight(0.32);
                else launchCmds.setLight(0);
            } if (lightTimer.seconds() > 1) lightTimer.reset();

            indexerCmds.getDetectedColor();
        }

        if (indexerCmds.isFull() && !hasBeenFull)
        {
            hasBeenFull = true;
            gamepad1.rumble(500);
        } if (!indexerCmds.isFull()) hasBeenFull = false;

        controls();

        Draw.drawDebug(follower);
        telemetry.addData("Time", gameTimer.seconds());
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
        if (gamepad1.backWasPressed() || gamepad1.shareWasPressed()) indexerCmds.clearContents();
        if (gamepad1.startWasPressed() || gamepad1.optionsWasPressed()) follower.setPose(new Pose(72, 144, Math.toRadians(90)));
        if (gamepad1.aWasPressed() || gamepad1.crossWasPressed())
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
        if (gamepad1.aWasReleased() || gamepad1.crossWasReleased())
        {
            autoPower = true;
            humanPlayer = false;
            idlePower = 0;
            intakeCmds.intakeForward();
            intakeCmds.intakeOn();
            intakeCmds.hammerPassive();
        }
        if (gamepad1.bWasPressed() || gamepad1.circleWasPressed()) indexerCmds.prevSlot();
        if (gamepad1.xWasPressed() || gamepad1.squareWasPressed()) indexerCmds.nextSlot();
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
        if (gamepad1.guideWasPressed() || gamepad1.psWasPressed()) basing = true;

        if (gamepad2.backWasPressed() || gamepad2.shareWasPressed())
        {
            intakeCmds.intakeReverse();
            intakeCmds.intakeOn();
        }
        if (gamepad2.backWasReleased() || gamepad2.shareWasReleased())
        {
            intakeCmds.intakeForward();
            intakeCmds.intakeOn();
        }
        if (gamepad2.aWasPressed() || gamepad2.crossWasPressed()) schedule(launchCmds.shootColor(gamepad2, DetectedColor.GREEN));
        if (gamepad2.bWasPressed() || gamepad2.circleWasPressed()) schedule(launchCmds.shootMotif(gamepad2, motifTranslated));
        if (gamepad2.xWasPressed() || gamepad2.squareWasPressed()) schedule(launchCmds.shootColor(gamepad2, DetectedColor.PURPLE));
        if (gamepad2.yWasPressed() || gamepad2.triangleWasPressed())
        {
            schedule(launchCmds.shootRapid(gamepad2));
            telemetry.speak("OoOoOoOooooOOoOoOoOooooOOoOoOoOooooOOoOoOoOooooOOoOoOoOooooO");
            telemetry.update();
        }
        if (gamepad2.dpadLeftWasPressed()) offset += 2.5;
        if (gamepad2.dpadRightWasPressed()) offset -= 2.5;
        if (gamepad2.dpadUpWasPressed()) angle += 0.01;
        if (gamepad2.dpadDownWasPressed()) angle -= 0.01;
        if (gamepad2.leftBumperWasPressed()) schedule(launchCmds.shootColor(gamepad2, indexerCmds.getIntakeSlotColor()));
        if (gamepad2.rightBumperWasPressed()) turretCmds.spinUpToVelocity();
        if (gamepad2.right_trigger > 0.1) turretCmds.deactivateLauncher();
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

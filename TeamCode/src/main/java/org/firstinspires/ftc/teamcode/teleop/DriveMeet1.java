package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedropathing.Draw;
import org.firstinspires.ftc.teamcode.subsystem.commands.IndexerCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

import org.firstinspires.ftc.teamcode.util.DetectedColor;
import org.firstinspires.ftc.teamcode.util.GlobalState;
import org.firstinspires.ftc.teamcode.util.MotifUtil;
import org.firstinspires.ftc.teamcode.util.Slot;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

@TeleOp(name = "DriveMeet1")
@Configurable
public class DriveMeet1 extends CommandOpMode {

    private DcMotorEx intake;

    private IndexerCommands indexerCmds;
    private TurretCommands turretCmds;

    private Follower follower;

    private AllianceColor allianceColor;
    private double intakePower = 0.0;
    private double launchPower = 1.0;
    private boolean autoAim = true;
    private boolean isRobotCentric = false;
    private double launchAngleDeg = 25.0;
    private Slot currentSlot = Slot.FIRST;
    private LimelightCommands.Motif motif = LimelightCommands.Motif.UNKNOWN;

    private NormalizedColorSensor cs;
    private float[] hHistory = new float[10];
    private float[] vHistory = new float[10];
    private int colorIdx = 0;
    private int colorCount = 0;

    private enum AvgDetectedColor { NONE, PURPLE, GREEN }
    private AvgDetectedColor lastDetected = AvgDetectedColor.NONE;

    private boolean started = false;
    private boolean indexerBusy = false;

    @Override
    public void initialize() {
        cs = hardwareMap.get(NormalizedColorSensor.class, "color");
        cs.setGain(200.0f);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        if (GlobalState.teleOpStartPose != null) {
            follower.setStartingPose(GlobalState.teleOpStartPose);
        } else {
            follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        }
        if (GlobalState.alliance != null) {
            allianceColor = GlobalState.alliance;
        }
        else {
            allianceColor = AllianceColor.RED;
        }
        if (GlobalState.lastMotif != LimelightCommands.Motif.UNKNOWN) {
            motif = GlobalState.lastMotif;
        }
        else {
            motif = LimelightCommands.Motif.PPG;
        }
        follower.update();

        indexerCmds = new IndexerCommands(hardwareMap);
        turretCmds = new TurretCommands(hardwareMap);

        PanelsConfigurables.INSTANCE.refreshClass(DriveMeet1.class);
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        telemetry.addLine("DriveMeet1 — Initialized");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        // init_loop()
        if (!isStarted()) {
            if (gamepad1.xWasPressed()) allianceColor = AllianceColor.BLUE;
            if (gamepad1.bWasPressed()) allianceColor = AllianceColor.RED;

            if (gamepad1.startWasPressed()) {
                follower.setPose(new Pose(72, 72, Math.toRadians(90)));
            }
            if (gamepad1.backWasPressed()) turretCmds.zeroHere();

            if (gamepad1.dpadUpWasPressed()) indexerCmds.new HammerUp().initialize();
            if (gamepad1.dpadDownWasPressed()) indexerCmds.new HammerDown().initialize();

            telemetry.addData("Alliance (g1x, g1b)", allianceColor);
            telemetry.addData("LaunchAngle", "%.1f", launchAngleDeg);
            telemetry.addData("CurrentSlot", currentSlot);
            telemetry.addData("Motif", motif);
            telemetry.update();
            return;
        }

        // start()
        if (!started) {
            turretCmds.setLaunchAngle(launchAngleDeg).execute();
            indexerCmds.indexer.setPosition(SLOT1_HOLD);
            indexerCmds.hammer.setPosition(HAMMER_REST);
            intakePower = 1.0;
            follower.startTeleopDrive();
            started = true;
        }

        follower.update();

        follower.setTeleOpDrive(
            -gamepad1.left_stick_y,
            gamepad1.left_stick_x * (allianceColor == AllianceColor.RED ? -1 : 1),
            -gamepad1.right_stick_x,
            isRobotCentric
        );

        // exclude gamepad1.a in case we need to add gamepad during the match
        if (gamepad1.startWasPressed() && !gamepad1.a) {
            follower.setPose(new Pose(72, 72, Math.toRadians(90)));
        }
        if (gamepad1.backWasPressed()) {
            isRobotCentric = !isRobotCentric;
        }

        double xDist = 0, yDist = 0;
        if (allianceColor == AllianceColor.RED) {
            xDist = RED_GOAL_X - (follower.getPose().getX() - 72);
            yDist = RED_GOAL_Y - (follower.getPose().getY() - 72) + 6;
        }
        else if (allianceColor == AllianceColor.BLUE) {
            xDist = BLUE_GOAL_X - (follower.getPose().getX() - 72);
            yDist = BLUE_GOAL_Y - (follower.getPose().getY() - 72) + 6;
        }
        double angleFromPosDeg = Math.toDegrees(Math.atan2(xDist, yDist));
        double baseTargetDeg = Math.toDegrees(follower.getPose().getHeading()) - 90 + angleFromPosDeg;

        if (gamepad2.leftStickButtonWasPressed()) autoAim = !autoAim;

        if (autoAim) {
            CommandBase step = turretCmds.setTarget(baseTargetDeg);
            step.initialize();
            step.execute();
            if (step.isFinished()) { step.end(false); }
        }

        if (gamepad2.left_stick_x != 0) {
            CommandBase c = turretCmds.AdjustTarget(gamepad2.left_stick_x / 2);
            c.initialize(); c.execute(); if (c.isFinished()) c.end(false);
        }
        if (gamepad2.startWasPressed()) {
            indexerCmds.clearAllSlotColors();
        }

        if (gamepad2.dpadUpWasPressed()) {
            launchAngleDeg = Range.clip(launchAngleDeg + 5, 15.0, 60.0);
            turretCmds.setLaunchAngle(launchAngleDeg).execute();
        }
        if (gamepad2.dpadDownWasPressed()) {
            launchAngleDeg = Range.clip(launchAngleDeg - 5, 15.0, 60.0);
            turretCmds.setLaunchAngle(launchAngleDeg).execute();
        }
        if (gamepad2.dpadLeftWasPressed()) {
            launchPower = Range.clip(launchPower + 0.05, 0, 1.0);
        }
        if (gamepad2.dpadRightWasPressed()) {
            launchPower = Range.clip(launchPower - 0.05, 0, 1.0);
        }

        if (gamepad2.xWasPressed()) turretCmds.toggleLauncher().initialize();
        if (gamepad2.aWasPressed()) scheduleShootColor(DetectedColor.GREEN);
        if (gamepad2.bWasPressed()) scheduleShootColor(DetectedColor.PURPLE);

        if (gamepad2.yWasPressed() && !indexerBusy) {
            schedule(shootMotifFromDetection(launchPower));
        }

        if (gamepad2.rightBumperWasPressed() && !indexerBusy) {
            Slot nextSlot = nextOf(currentSlot);
            schedule(spinToIntakeAndUpdate(nextSlot));
        }

        if (gamepad1.dpadDownWasPressed())  intakePower =  INTAKE_ACTIVE_POWER;
        if (gamepad1.dpadUpWasPressed())    intakePower = -INTAKE_ACTIVE_POWER;
        if (gamepad1.dpadLeftWasPressed())  intakePower = 0.0;
        intake.setPower(intakePower);

        NormalizedRGBA rgba = cs.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(rgba.toColor(), hsv);
        float H = hsv[0];
        float V = hsv[2];

        hHistory[colorIdx] = H;
        vHistory[colorIdx] = V;
        colorIdx = (colorIdx + 1) % 10;
        if (colorCount < 10) colorCount++;

        float hAvg = 0f;
        float vAvg = 0f;
        for (int i = 0; i < colorCount; i++) {
            hAvg += hHistory[i];
            vAvg += vHistory[i];
        }
        hAvg /= colorCount;
        vAvg /= colorCount;

        AvgDetectedColor currentDetected;
        if (vAvg <= 0.23f) {
            currentDetected = AvgDetectedColor.NONE;
        } else if (165.0f < hAvg && hAvg < 210.0f) {
            currentDetected = AvgDetectedColor.PURPLE;
        } else if (152.0f < hAvg && hAvg < 164.0f) {
            currentDetected = AvgDetectedColor.GREEN;
        } else {
            currentDetected = AvgDetectedColor.NONE;
        }

        if (currentDetected != AvgDetectedColor.NONE
            && lastDetected == AvgDetectedColor.NONE
            && !indexerBusy) {
            if (currentDetected == AvgDetectedColor.GREEN) {
                indexerCmds.new SetSlotColor(currentSlot, DetectedColor.GREEN).initialize();
            } else if (currentDetected == AvgDetectedColor.PURPLE) {
                indexerCmds.new SetSlotColor(currentSlot, DetectedColor.PURPLE).initialize();
            }

            boolean allFull =
                indexerCmds.getSlotColor(Slot.FIRST) != DetectedColor.UNKNOWN &&
                    indexerCmds.getSlotColor(Slot.SECOND) != DetectedColor.UNKNOWN &&
                    indexerCmds.getSlotColor(Slot.THIRD) != DetectedColor.UNKNOWN;

            if (!allFull) {
                Slot nextSlot = nextOf(currentSlot);
                schedule(spinToIntakeAndUpdate(nextSlot));
            }
        }

        lastDetected = currentDetected;

        Draw.drawDebug(follower);

        telemetry.addData("Alliance", allianceColor);
        telemetry.addData("Pose (mm, deg)", "x=%.1f  y=%.1f  h=%.1f",
            follower.getPose().getX(), follower.getPose().getY(), 90 - Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Intake Pwr", "%.2f", intakePower);
        telemetry.addData("LaunchAngle", "%.1f", launchAngleDeg);
        telemetry.addData("CurrentSlot", currentSlot);
        telemetry.addData("AutoAim", autoAim ? "ON" : "OFF");
        telemetry.addData("angle from pos", angleFromPosDeg);
        telemetry.addData("base target", baseTargetDeg);
        telemetry.addData("Color H/V avg", "H=%.1f  V=%.3f", hAvg, vAvg);
        telemetry.addData("AvgDetected", currentDetected);
        telemetry.addData("Slot1", indexerCmds.getSlotColor(Slot.FIRST));
        telemetry.addData("Slot2", indexerCmds.getSlotColor(Slot.SECOND));
        telemetry.addData("Slot3", indexerCmds.getSlotColor(Slot.THIRD));

        telemetry.update();
    }

    private static Slot nextOf(Slot s) {
        switch (s) {
            case FIRST:  return Slot.SECOND;
            case SECOND: return Slot.THIRD;
            case THIRD:  return Slot.FIRST;
            default:     return Slot.FIRST;
        }
    }

    private CommandBase spinToShootSlot(Slot slotToShoot) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> indexerBusy = true),
            indexerCmds.new SpinToShoot(slotToShoot),
            new InstantCommand(() -> indexerBusy = false)
        );
    }

    private CommandBase spinToIntakeAndUpdate(Slot nextSlot) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> indexerBusy = true),
            indexerCmds.new SpinToIntake(nextSlot),
            new InstantCommand(() -> {
                currentSlot = nextSlot;
                indexerBusy = false;
                colorCount = 0;
                colorIdx = 0;
                lastDetected = AvgDetectedColor.NONE;
            })
        );
    }

    private CommandBase shootColor(DetectedColor color) {
        Slot slotToShoot = indexerCmds.findFirstSlotWithColor(color);
        if (slotToShoot == null) return new InstantCommand(() -> {});
        Slot nextSlot = nextOf(slotToShoot);

        return new SequentialCommandGroup(
            new InstantCommand(() -> indexerBusy = true),
            new ParallelCommandGroup(
                indexerCmds.new SpinToShoot(slotToShoot),
                turretCmds.activateLauncher()
            ),
            indexerCmds.new HammerUp(),
            indexerCmds.new HammerDown(),
            indexerCmds.new SpinToIntake(nextSlot),
            new InstantCommand(() -> {
                currentSlot = nextSlot;
                indexerCmds.clearSlotColor(slotToShoot);
                indexerBusy = false;
                colorCount = 0;
                colorIdx = 0;
            })
        );
    }

    private CommandBase shoot(Slot slot, double power) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                indexerCmds.new SpinToShoot(slot),
                turretCmds.activateLauncher(power)
            ),
            indexerCmds.new HammerUp(),
            indexerCmds.new HammerDown()
        );
    }

    //TODO: duplicated func. on auto
    private Slot[] pickSlotsForMotif(DetectedColor[] motifColors) {
        DetectedColor[] slotColors = new DetectedColor[]{
            indexerCmds.getSlotColor(Slot.FIRST),
            indexerCmds.getSlotColor(Slot.SECOND),
            indexerCmds.getSlotColor(Slot.THIRD)
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

    private CommandBase shootMotifFromDetection(double power) {
        return new CommandBase() {
            private CommandBase inner;

            @Override
            public void initialize() {
                DetectedColor[] colors = MotifUtil.motifToColors(motif);

                if (colors == null || colors.length < 3) {
                    inner = new InstantCommand(() -> {});
                    inner.initialize();
                    return;
                }

                Slot[] slots = pickSlotsForMotif(colors);

                CommandBase c0 = (slots[0] != null) ? shoot(slots[0], power) : new InstantCommand(() -> {});
                CommandBase c1 = (slots[1] != null) ? shoot(slots[1], power) : new InstantCommand(() -> {});
                CommandBase c2 = (slots[2] != null) ? shoot(slots[2], power) : new InstantCommand(() -> {});

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
        };
    }

    private void scheduleShootColor(DetectedColor color) {
        if (!indexerBusy) {
            schedule(shootColor(color));
        }
    }
}

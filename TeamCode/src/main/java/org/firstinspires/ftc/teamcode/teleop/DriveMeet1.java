package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystem.commands.IndexerCommands;
import org.firstinspires.ftc.teamcode.subsystem.commands.TurretCommands;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

import org.firstinspires.ftc.teamcode.util.DetectedColor;
import org.firstinspires.ftc.teamcode.util.Slot;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

@TeleOp(name = "DriveMeet1")
@Configurable
public class DriveMeet1 extends OpMode {

    // Hardware
    private DcMotorEx intake;

    // Subsystem command wrappers
    private IndexerCommands indexerCmds;
    private TurretCommands turretCmds;

    // PP set-up
    private Follower follower;

    // Driver state
    private AllianceColor allianceColor = AllianceColor.RED;
    private double intakePower = 0.0;
    private boolean autoAim = true;
    private double launchAngleDeg = 25.0;
    private Slot currentSlot = Slot.FIRST;

    private CommandBase activeIndexerCmd = null;


    // Color sensor, buffer for avg. calculation
    private NormalizedColorSensor cs;
    private float[] hHistory = new float[10]; // last 10 H-values
    private float[] vHistory = new float[10]; // last 10 V-values
    private int colorIdx = 0; // buffer index
    private int colorCount = 0;

    private enum AvgDetectedColor { NONE, PURPLE, GREEN }
    private AvgDetectedColor lastDetected = AvgDetectedColor.NONE;


    @Override
    public void init() {
        cs = hardwareMap.get(NormalizedColorSensor.class, "color");
        cs.setGain(200.0f);

        // Drivetrain motors
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();

        // Command wrappers
        indexerCmds = new IndexerCommands(hardwareMap);
        turretCmds  = new TurretCommands(hardwareMap);

        PanelsConfigurables.INSTANCE.refreshClass(DriveMeet1.class);
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        telemetry.addLine("DriveMeet1 — Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) allianceColor = AllianceColor.BLUE;
        if (gamepad1.bWasPressed()) allianceColor = AllianceColor.RED;

        if (gamepad1.dpadUpWasPressed()) indexerCmds.new HammerUp().initialize();
        if (gamepad1.dpadDownWasPressed()) indexerCmds.new HammerDown().initialize();

        telemetry.addData("Alliance", allianceColor);
        telemetry.addData("AutoAim (g2 LS click)", autoAim ? "ON" : "OFF");
        telemetry.addData("LaunchAngle", "%.1f", launchAngleDeg);
        telemetry.addData("CurrentSlot", currentSlot);
        telemetry.update();
    }

    @Override
    public void start() {
        turretCmds.zero().initialize();
        turretCmds.setLaunchAngle(launchAngleDeg).initialize();
        indexerCmds.indexer.setPosition(SLOT1_HOLD);
        indexerCmds.hammer.setPosition(HAMMER_REST);

        intakePower = 1.0;

        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        follower.setTeleOpDrive(
            -gamepad1.left_stick_x,
            gamepad1.left_stick_y * (allianceColor == AllianceColor.RED ? 1 : -1),
            -gamepad1.right_stick_x,
            false
        );

        // Reset heading
        if (gamepad1.startWasPressed()) {
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
        }

        // Turret auto-aim calculation
        double xDist = 0, yDist = 0;
        if (allianceColor == AllianceColor.RED) {
            xDist = RED_GOAL_X - follower.getPose().getX();
            yDist = RED_GOAL_Y - follower.getPose().getY() + 6; // +6 inch offset
        }
        else if (allianceColor == AllianceColor.BLUE) {
            xDist = BLUE_GOAL_X - follower.getPose().getX();
            yDist = BLUE_GOAL_Y - follower.getPose().getY() + 6; // +6 inch offset
        }
        double angleFromPosDeg = Math.toDegrees(Math.atan2(xDist, yDist));
        double baseTargetDeg = Math.toDegrees(follower.getPose().getHeading()) + angleFromPosDeg;

        if (gamepad2.leftStickButtonWasPressed()) autoAim = !autoAim;

        if (autoAim) {
            CommandBase step = turretCmds.setTarget(baseTargetDeg);
            step.initialize();
            step.execute();
            if (step.isFinished()) { step.end(false); }
        }

        if (gamepad2.left_stick_x != 0) {
            CommandBase c = turretCmds.AdjustTarget(-gamepad2.left_stick_x / 2);
            c.initialize(); c.execute(); if (c.isFinished()) c.end(false);
        }

        if (gamepad2.dpadUpWasPressed()) {
            launchAngleDeg = Range.clip(launchAngleDeg + 10.0, 20.0, 50.0);
            turretCmds.setLaunchAngle(launchAngleDeg).initialize();
        }
        if (gamepad2.dpadDownWasPressed()) {
            launchAngleDeg = Range.clip(launchAngleDeg - 10.0, 20.0, 50.0);
            turretCmds.setLaunchAngle(launchAngleDeg).initialize();
        }

        if (gamepad2.xWasPressed()) turretCmds.toggleLauncher().initialize();
        if (gamepad2.aWasPressed()) scheduleShootColor(DetectedColor.GREEN);
        if (gamepad2.bWasPressed()) scheduleShootColor(DetectedColor.PURPLE);

        if (gamepad2.yWasPressed() && activeIndexerCmd == null) {
            activeIndexerCmd = indexerCmds.new SpinToShoot(currentSlot);
            activeIndexerCmd.initialize();
        }

        if (gamepad2.rightBumperWasPressed() && activeIndexerCmd == null) {
            Slot nextSlot = nextOf(currentSlot);
            activeIndexerCmd = indexerCmds.new SpinToIntake(nextSlot);
            activeIndexerCmd.initialize();
            currentSlot = nextSlot;

            colorCount = 0;
            colorIdx = 0;
            lastDetected = AvgDetectedColor.NONE;
        }

        if (activeIndexerCmd != null) {
            activeIndexerCmd.execute();
            if (activeIndexerCmd.isFinished()) {
                activeIndexerCmd.end(false);
                activeIndexerCmd = null;
            }
        }

        // Intake motor control
        if (gamepad1.dpadDownWasPressed())  intakePower =  INTAKE_ACTIVE_POWER;
        if (gamepad1.dpadUpWasPressed())    intakePower = -INTAKE_ACTIVE_POWER;
        if (gamepad1.dpadLeftWasPressed())  intakePower = 0.0;
        intake.setPower(intakePower);

        // Color sensor
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

        // Determine color
        AvgDetectedColor currentDetected;
        if (vAvg <= 0.23f) {
            currentDetected = AvgDetectedColor.NONE;     // UNKNOWN
        } else if (165.0f < hAvg && hAvg < 210.0f) {
            currentDetected = AvgDetectedColor.PURPLE;   // PURPLE
        } else if (152.0f < hAvg && hAvg < 164.0f) {
            currentDetected = AvgDetectedColor.GREEN;    // GREEN
        } else {
            currentDetected = AvgDetectedColor.NONE;     // 범위 밖 → UNKNOWN 취급
        }

        if (currentDetected != AvgDetectedColor.NONE
            && lastDetected == AvgDetectedColor.NONE
            && activeIndexerCmd == null) {

            if (currentDetected == AvgDetectedColor.GREEN) {
                indexerCmds.setSlotColor(currentSlot, DetectedColor.GREEN);
            } else if (currentDetected == AvgDetectedColor.PURPLE) {
                indexerCmds.setSlotColor(currentSlot, DetectedColor.PURPLE);
            }

            boolean allFull =
                indexerCmds.getSlotColor(Slot.FIRST) != DetectedColor.UNKNOWN &&
                    indexerCmds.getSlotColor(Slot.SECOND) != DetectedColor.UNKNOWN &&
                    indexerCmds.getSlotColor(Slot.THIRD) != DetectedColor.UNKNOWN;

            // Prevent magazine keep spinning
            if (!allFull) {
                Slot nextSlot = nextOf(currentSlot);
                activeIndexerCmd = indexerCmds.new SpinToIntake(nextSlot);
                activeIndexerCmd.initialize();
                currentSlot = nextSlot;

                colorCount = 0;
                colorIdx = 0;
            }
        }

        lastDetected = currentDetected;


        // Telemetry
        telemetry.addData("Alliance", allianceColor);
        telemetry.addData("Pose (mm, deg)", "x=%.1f  y=%.1f  h=%.1f",
            follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
        telemetry.addData("Intake Pwr", "%.2f", intakePower);
        telemetry.addData("LaunchAngle", "%.1f", launchAngleDeg);
        telemetry.addData("CurrentSlot", currentSlot);
        telemetry.addData("AutoAim", autoAim ? "ON" : "OFF");
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

    private void scheduleShootColor(DetectedColor color) {
        if (activeIndexerCmd != null) return;

        Slot slotToShoot = indexerCmds.findFirstSlotWithColor(color);
        if (slotToShoot == null) {
            telemetry.addData("ShootColor", "No slot with color %s", color);
            return;
        }

        Slot nextSlot = nextOf(slotToShoot);

        activeIndexerCmd = new SequentialCommandGroup(
            new ParallelCommandGroup(
                indexerCmds.new SpinToShoot(slotToShoot),
                turretCmds.activateLauncher()
            ),
            indexerCmds.new HammerUp(),
            indexerCmds.new HammerDown(),
            indexerCmds.new SpinToIntake(nextSlot)
        );

        activeIndexerCmd.initialize();

        indexerCmds.clearSlotColor(slotToShoot);

        currentSlot = nextSlot;
    }

}

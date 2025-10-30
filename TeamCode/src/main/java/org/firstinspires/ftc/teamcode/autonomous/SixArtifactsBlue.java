package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Magazine;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@Autonomous(name = "SixArtifactsBlue")
public class SixArtifactsBlue extends LinearOpMode {
    private Turret turret;
    private Magazine mag;
    private DcMotorEx intake;
    private Paths paths;

    // FSM
    private enum State {
        // Volley #1 (same style as your working ThreeArtifacts)
        START, PATH1_AND_PRIME, WAIT_PATH1_AND_TURRET,
        FIRE_1, WAIT_FIRE_1, FIRE_2, WAIT_FIRE_2, FIRE_3, WAIT_FIRE_3,

        // Collection (Paths 2–5)
        PATH2_SETUP, WAIT_PATH2,
        PATH3_SETUP, WAIT_PATH3_THEN_SLOT2,
        PATH4_SETUP, WAIT_PATH4_THEN_SLOT3,
        PATH5_SETUP, WAIT_PATH5_MARK,

        // Return & Volley #2 (Path 6)
        PATH6_AND_PRIME, WAIT_PATH6_AND_TURRET,
        FIRE_A, WAIT_FIRE_A, FIRE_B, WAIT_FIRE_B, FIRE_C, WAIT_FIRE_C,

        DONE
    }
    private State state = State.START;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Tunables
    private static final double TURRET_TOL_DEG = 2.0;
    private static final double PATH_TIMEOUT_S = 4.0;
    private static final double SETTLE_S       = 1.5;  // short dwell
    private static final double INTAKE_POWER   = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        // Subsystems
        turret = new Turret(hardwareMap);
        turret.zeroHere();
        mag    = new Magazine(hardwareMap);
        mag.init();
        // Start neutral; we mark as we pick up
        mag.setColors(Magazine.DetectedColor.UNKNOWN, Magazine.DetectedColor.UNKNOWN, Magazine.DetectedColor.UNKNOWN);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(0);

        // Pedro follower + paths
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(28.0, 132.0, Math.toRadians(144)));
        paths = new Paths(follower);

        telemetry.addLine("SiX SevEn");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        state = State.PATH1_AND_PRIME;
        stateTimer.reset();

        while (opModeIsActive()) {
            // periodic updates
            follower.update();
            turret.update();
            mag.update();

            switch (state) {
                /* ---------------- Volley #1 (Path1) ---------------- */
                case PATH1_AND_PRIME:
                    follower.followPath(paths.Path1, true);
                    turret.activateLauncher();
                    turret.setLaunchAngle(42.5);
                    turret.setTarget(4);
                    intake.setPower(INTAKE_POWER);
                    state = State.WAIT_PATH1_AND_TURRET;
                    stateTimer.reset();
                    break;

                case WAIT_PATH1_AND_TURRET: {
                    boolean pathDone   = !follower.isBusy() || stateTimer.seconds() > PATH_TIMEOUT_S;
                    boolean turretDone = Math.abs(turret.getErrorDeg()) <= TURRET_TOL_DEG || stateTimer.seconds() >PATH_TIMEOUT_S;
                    if (pathDone && turretDone && stateTimer.seconds() > SETTLE_S) {
                        state = State.FIRE_1;
                        stateTimer.reset();
                    }
                } break;

                case FIRE_1:
                    if (!mag.isBusy()) {
                        if (mag.tryStartLaunch(Magazine.Slot.FIRST)) {
                            state = State.WAIT_FIRE_1;
                            stateTimer.reset();
                        }
                    }
                    break;

                case WAIT_FIRE_1:
                    if (!mag.isBusy() && stateTimer.seconds() > SETTLE_S) {
                        state = State.FIRE_2;
                        stateTimer.reset();
                    }
                    break;

                case FIRE_2:
                    if (!mag.isBusy()) {
                        if (mag.tryStartLaunch(Magazine.Slot.SECOND)) {
                            state = State.WAIT_FIRE_2;
                            stateTimer.reset();
                        }
                    }
                    break;

                case WAIT_FIRE_2:
                    if (!mag.isBusy() && stateTimer.seconds() > SETTLE_S) {
                        state = State.FIRE_3;
                        stateTimer.reset();
                    }
                    break;

                case FIRE_3:
                    if (!mag.isBusy()) {
                        if (mag.tryStartLaunch(Magazine.Slot.THIRD)) {
                            state = State.WAIT_FIRE_3;
                            stateTimer.reset();
                        }
                    }
                    break;

                case WAIT_FIRE_3:
                    if (!mag.isBusy() && stateTimer.seconds() > SETTLE_S) {
                        state = State.PATH2_SETUP;
                        stateTimer.reset();
                    }
                    break;

                /* ------------- Path2: deactivate, slot1, intake ON ------------- */
                case PATH2_SETUP:
                    follower.followPath(paths.Path2, true);
                    turret.deactivateLauncher();
                    if (!mag.isBusy()) mag.setSlot(Magazine.Slot.FIRST);
                    intake.setPower(INTAKE_POWER);
                    state = State.WAIT_PATH2;
                    stateTimer.reset();
                    break;

                case WAIT_PATH2:
                    if ((!follower.isBusy() || stateTimer.seconds() > PATH_TIMEOUT_S) &&
                        stateTimer.seconds() > SETTLE_S) {
                        state = State.PATH3_SETUP;
                        stateTimer.reset();
                    }
                    break;

                /* ------------- Path3: wait, slot2, mark slot1=PURPLE ------------- */
                case PATH3_SETUP:
                    follower.followPath(paths.Path3, true);
                    state = State.WAIT_PATH3_THEN_SLOT2;
                    stateTimer.reset();
                    break;

                case WAIT_PATH3_THEN_SLOT2:
                    if (stateTimer.seconds() > SETTLE_S) {
                        if (!mag.isBusy()) mag.setSlot(Magazine.Slot.SECOND);
                        markSlotColor(Magazine.Slot.FIRST, Magazine.DetectedColor.PURPLE);
                        if (!follower.isBusy()) {
                            state = State.PATH4_SETUP;
                            stateTimer.reset();
                        }
                    }
                    break;

                /* ------------- Path4: wait, slot3, mark slot2=PURPLE ------------- */
                case PATH4_SETUP:
                    follower.followPath(paths.Path4, true);
                    state = State.WAIT_PATH4_THEN_SLOT3;
                    stateTimer.reset();
                    break;

                case WAIT_PATH4_THEN_SLOT3:
                    if (stateTimer.seconds() > SETTLE_S) {
                        if (!mag.isBusy()) mag.setSlot(Magazine.Slot.THIRD);
                        markSlotColor(Magazine.Slot.SECOND, Magazine.DetectedColor.PURPLE);
                        if (!follower.isBusy()) {
                            state = State.PATH5_SETUP;
                            stateTimer.reset();
                        }
                    }
                    break;

                /* ------------- Path5: mark slot3=GREEN ------------- */
                case PATH5_SETUP:
                    follower.followPath(paths.Path5, true);
                    markSlotColor(Magazine.Slot.THIRD, Magazine.DetectedColor.GREEN);
                    state = State.WAIT_PATH5_MARK;
                    stateTimer.reset();
                    break;

                case WAIT_PATH5_MARK:
                    if (!follower.isBusy() && stateTimer.seconds() > SETTLE_S) {
                        state = State.PATH6_AND_PRIME;
                        stateTimer.reset();
                    }
                    break;

                /* ------------- Path6: return and prime volley #2 ------------- */
                case PATH6_AND_PRIME:
                    follower.followPath(paths.Path6, true);
                    turret.activateLauncher();
                    turret.setLaunchAngle(42.5);
                    turret.setTarget(4);
                    state = State.WAIT_PATH6_AND_TURRET;
                    stateTimer.reset();
                    break;

                case WAIT_PATH6_AND_TURRET: {
                    boolean pathDone   = !follower.isBusy() || stateTimer.seconds() > PATH_TIMEOUT_S;
                    boolean turretDone = Math.abs(turret.getErrorDeg()) <= TURRET_TOL_DEG || stateTimer.seconds() > PATH_TIMEOUT_S;
                    if (pathDone && turretDone && stateTimer.seconds() > SETTLE_S) {
                        intake.setPower(0); // stop intake before firing
                        state = State.FIRE_A;
                        stateTimer.reset();
                    }
                } break;

                /* ------------- Volley #2 by slot (deterministic) ------------- */
                case FIRE_A:
                    if (!mag.isBusy()) {
                        if (mag.tryStartLaunch(Magazine.Slot.FIRST)) {
                            state = State.WAIT_FIRE_A;
                            stateTimer.reset();
                        }
                    }
                    break;

                case WAIT_FIRE_A:
                    if (!mag.isBusy() && stateTimer.seconds() > SETTLE_S) {
                        state = State.FIRE_B;
                        stateTimer.reset();
                    }
                    break;

                case FIRE_B:
                    if (!mag.isBusy()) {
                        if (mag.tryStartLaunch(Magazine.Slot.SECOND)) {
                            state = State.WAIT_FIRE_B;
                            stateTimer.reset();
                        }
                    }
                    break;

                case WAIT_FIRE_B:
                    if (!mag.isBusy() && stateTimer.seconds() > SETTLE_S) {
                        state = State.FIRE_C;
                        stateTimer.reset();
                    }
                    break;

                case FIRE_C:
                    if (!mag.isBusy()) {
                        if (mag.tryStartLaunch(Magazine.Slot.THIRD)) {
                            state = State.WAIT_FIRE_C;
                            stateTimer.reset();
                        }
                    }
                    break;

                case WAIT_FIRE_C:
                    if (!mag.isBusy() && stateTimer.seconds() > SETTLE_S) {
                        state = State.DONE;
                        stateTimer.reset();
                    }
                    break;

                case DONE:
                    turret.deactivateLauncher();
                    requestOpModeStop();
                    break;
            }

            // minimal telemetry
            telemetry.addData("State", state);
            telemetry.addData("Path busy", follower.isBusy());
            telemetry.addData("Shooter vel", turret.launchMotor.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("Turret err", "%.1f", turret.getErrorDeg());
            telemetry.addData("Mag busy", mag.isBusy());
            telemetry.update();
        }
    }

    /** Mark a single slot’s color without changing the others. */
    private void markSlotColor(Magazine.Slot slot, Magazine.DetectedColor color) {
        Magazine.DetectedColor c1 = mag.getLastColor(Magazine.Slot.FIRST);
        Magazine.DetectedColor c2 = mag.getLastColor(Magazine.Slot.SECOND);
        Magazine.DetectedColor c3 = mag.getLastColor(Magazine.Slot.THIRD);
        switch (slot) {
            case FIRST:  c1 = color; break;
            case SECOND: c2 = color; break;
            case THIRD:  c3 = color; break;
        }
        mag.setColors(c1, c2, c3);
    }

    /** Paths 1..6 exactly as specified */
    public static class Paths {
        public final PathChain Path1, Path2, Path3, Path4, Path5, Path6;

        public Paths(Follower follower) {
            Path1 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(28.000, 132.000), new Pose(56.000, 96.000)))
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(135))
                .build();

            Path2 = follower
                .pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(56.000, 96.000),
                    new Pose(54.000, 84.000),
                    new Pose(45.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

            Path3 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(45.000, 84.000), new Pose(37.500, 84.000)))
                .setTangentHeadingInterpolation()
                .build();

            Path4 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(37.500, 84.000), new Pose(30.000, 84.000)))
                .setTangentHeadingInterpolation()
                .build();

            Path5 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(30.000, 84.000), new Pose(22.500, 84.000)))
                .setTangentHeadingInterpolation()
                .build();

            Path6 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(22.500, 84.000), new Pose(56.000, 96.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
        }
    }
}

package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Magazine;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@Autonomous
public class ThreeArtifactsBlue extends LinearOpMode {
    private Turret turret;
    private Magazine mag;
    private PathChain path1;

    // simple FSM
    private enum State {
        START, PATH1_AND_PRIME, WAIT_PATH1_AND_TURRET, FIRE_1, WAIT_FIRE_1, FIRE_2, WAIT_FIRE_2, FIRE_3, WAIT_FIRE_3, DONE
    }
    private State state = State.START;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // tunables
    private static final double TURRET_TOL_DEG = 2.0;
    private static final double PATH_TIMEOUT_S = 6.0;   // safety
    private static final double SETTLE_S       = 0.2;   // short settle before first shot

    // helpers
    private boolean startedPath1 = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // init subsystems
        turret = new Turret(hardwareMap);
        mag    = new Magazine(hardwareMap);
        mag.init();
        mag.setColors(Magazine.DetectedColor.PURPLE, Magazine.DetectedColor.GREEN, Magazine.DetectedColor.PURPLE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(28.0, 132.0, Math.toRadians(144)));

        // build Path1
        path1 = follower.pathBuilder()
            .addPath(new BezierLine(new Pose(28.0, 132.0), new Pose(56.0, 96.0)))
            .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(135))
            .build();

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // zero at start, prime targets once we enter PATH1_AND_PRIME
        state = State.PATH1_AND_PRIME;
        stateTimer.reset();

        while (opModeIsActive()) {
            // periodic updates
            follower.update();
            turret.update();
            mag.update();

            switch (state) {
                case PATH1_AND_PRIME:
                    if (!startedPath1) {
                        follower.followPath(path1, true);
                        turret.activateLauncher();
                        turret.setLaunchAngle(40);
                        turret.setTarget(7);
                        startedPath1 = true;
                        stateTimer.reset();
                    }
                    // next: wait for path + turret settle
                    state = State.WAIT_PATH1_AND_TURRET;
                    break;

                case WAIT_PATH1_AND_TURRET:
                    boolean pathDone   = !follower.isBusy() || stateTimer.seconds() > PATH_TIMEOUT_S;
                    boolean turretDone = Math.abs(turret.getErrorDeg()) <= TURRET_TOL_DEG;
                    if (pathDone && turretDone && stateTimer.seconds() > SETTLE_S) {
                        state = State.FIRE_1;
                        stateTimer.reset();
                    }
                    break;

                case FIRE_1:
                    if (!mag.isBusy()) {
                        if (mag.tryStartLaunch(Magazine.Slot.FIRST)) {
                            state = State.WAIT_FIRE_1;
                            stateTimer.reset();
                        }
                        // else: stay in FIRE_1 and try again next loop
                    }
                    break;

                case WAIT_FIRE_1:
                    if (!mag.isBusy() && stateTimer.seconds() > 1) {
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
                    if (!mag.isBusy() && stateTimer.seconds() > 1) {
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
                    if (!mag.isBusy() && stateTimer.seconds() > 1) {
                        turret.setTarget(0);
                        state = State.DONE;
                        stateTimer.reset();
                    }
                    break;

                case DONE:
                    turret.deactivateLauncher();
                    requestOpModeStop();
                    break;

                default:
                    break;
            }

            // minimal telemetry
            telemetry.addData("State", state);
            telemetry.addData("Path busy", follower.isBusy());
            telemetry.addData("Turret err", "%.1f", turret.getErrorDeg());
            telemetry.addData("Mag busy", mag.isBusy());
            telemetry.update();
        }
    }
}

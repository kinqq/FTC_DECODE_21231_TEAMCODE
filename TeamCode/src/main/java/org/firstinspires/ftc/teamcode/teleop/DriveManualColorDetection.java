package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.Magazine;

import static org.firstinspires.ftc.teamcode.subsystem.Magazine.*;

import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

@TeleOp(name = "Drive Manual Color Detection")
@Configurable
public class DriveManualColorDetection extends OpMode {
    private GoBildaPinpointDriver odo;
    private DcMotorEx frontLeft, backLeft, frontRight, backRight, intake, launcher;
    private Magazine magazine;
    private Turret turret;
    private AllianceColor allianceColor = AllianceColor.RED;

    private DetectedColor slot1 = DetectedColor.UNKNOWN, slot2 = DetectedColor.UNKNOWN, slot3 = DetectedColor.UNKNOWN;
    private int idx = 1;
    private double intakePower = 0, launcherPower = 0, launcherPowerVariable = 0;
    private boolean intaking = false, atFullSpeed = false, forceLaunching = false, fieldCentric = true;

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(-48, -184.15, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        // Declare motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        magazine = new Magazine(hardwareMap);
        turret = new Turret(hardwareMap);

        //TODO: This is ONLY for BLUE side

        odo.setPosX(609.6, DistanceUnit.MM);
        odo.setPosY(406.4, DistanceUnit.MM);
        odo.setHeading(45, AngleUnit.DEGREES);

        PanelsConfigurables.INSTANCE.refreshClass(Drive.class);

        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        telemetry.addData("x", odo.getPosX(DistanceUnit.MM));
        telemetry.addData("y", odo.getPosY(DistanceUnit.MM));
        telemetry.addData("h", odo.getHeading(AngleUnit.DEGREES));
        telemetry.addLine("Initialized!");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) allianceColor = AllianceColor.BLUE;
        if (gamepad1.bWasPressed()) allianceColor = AllianceColor.RED;
        if (gamepad1.startWasPressed()) {
            odo.setPosX(-1600, DistanceUnit.MM);
            odo.setPosY(366 * (allianceColor == AllianceColor.BLUE ? 1 : -1), DistanceUnit.MM); // or -193
        }

        telemetry.addData("Alliance", allianceColor);
        telemetry.update();
    }

    @Override
    public void start() {
        turret.setLaunchAngle(42.5);
        intakePower = 1;

        magazine.init();
    }

    @Override
    public void loop() {
        odo.update();
        turret.update();

        if (gamepad1.startWasPressed()) odo.resetPosAndIMU();
        double heading = -odo.getHeading(AngleUnit.RADIANS) + Math.PI / 2 * (allianceColor == AllianceColor.BLUE ? 1 : 3); // blue: Pi/2 , red 3*pi/2
        if (!fieldCentric) heading = 0;

        double y = -gamepad1.left_stick_y; // Y stick is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double xSpeed = x * Math.cos(heading) - y * Math.sin(heading);
        double ySpeed = x * Math.sin(heading) + y * Math.cos(heading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (ySpeed + xSpeed + rx) / denominator;
        double backLeftPower = (ySpeed - xSpeed + rx) / denominator;
        double frontRightPower = (ySpeed - xSpeed - rx) / denominator;
        double backRightPower = (ySpeed + xSpeed - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        double xPos = odo.getPosX(DistanceUnit.MM);
        double yPos = odo.getPosY(DistanceUnit.MM);

        double xDistanceFromGoal = 0;
        double yDistanceFromGoal = 0;
        if (allianceColor == AllianceColor.RED) {
            xDistanceFromGoal = Constants.RED_GOAL_X - xPos + 150;
            yDistanceFromGoal = Constants.RED_GOAL_Y + yPos;
        }
        if (allianceColor == AllianceColor.BLUE) {
            xDistanceFromGoal = Constants.BLUE_GOAL_X - xPos + 150;
            yDistanceFromGoal = Constants.BLUE_GOAL_Y + yPos;
        }

        double distanceFromGoal = Math.sqrt(Math.pow(xDistanceFromGoal, 2) + Math.pow(yDistanceFromGoal, 2));
        double angleFromPosition = Math.atan2(yDistanceFromGoal, xDistanceFromGoal) / Math.PI * 180;
        double angleFromHeading = -odo.getHeading(AngleUnit.DEGREES);

        if (allianceColor == AllianceColor.RED) turret.setTarget(angleFromHeading - angleFromPosition);
        if (allianceColor == AllianceColor.BLUE) turret.setTarget(angleFromHeading - angleFromPosition);

        if (gamepad2.dpadLeftWasPressed()) magazine.prevSlot();
        if (gamepad2.dpadRightWasPressed()) magazine.nextSlot();
        if (gamepad2.dpadUpWasPressed()) turret.setLaunchAngle(turret.getLaunchAngle() + 2.5);
        if (gamepad2.dpadDownWasPressed()) turret.setLaunchAngle(turret.getLaunchAngle() - 2.5);

        if (gamepad1.dpadLeftWasPressed()) turret.adjustTarget(3);
        if (gamepad1.dpadRightWasPressed()) turret.adjustTarget(-3);
        if (gamepad1.dpadDownWasPressed()) intakePower = Constants.INTAKE_ACTIVE_POWER;
        if (gamepad1.dpadUpWasPressed()) intakePower = -Constants.INTAKE_ACTIVE_POWER;
        if (gamepad1.backWasPressed()) fieldCentric = !fieldCentric;
        if (gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5) intakePower = 0;

//        if (gamepad1.dpadLeftWasPressed()) {
//            intakePower = 0;
//            atFullSpeed = false;
//        }
        intake.setPower(intakePower);

//        double intakeVelocity = intake.getVelocity(AngleUnit.DEGREES);
//
//        if (intakeVelocity > 315 && !atFullSpeed) {
//            atFullSpeed = true;
//        }
//        if (atFullSpeed && !intaking & intakeVelocity < 260) {
//            intaking = true;
//        }
//        if (atFullSpeed && intaking && intakeVelocity > 315) {
//            magazine.nextSlot();
//            intaking = false;
//        }

        if (gamepad2.leftBumperWasPressed()) {
            switch (idx) {
                case 1:
                    slot1 = DetectedColor.PURPLE;
                    idx = 2;
                    break;
                case 2:
                    slot2 = DetectedColor.PURPLE;
                    idx = 3;
                    break;
                case 3:
                    slot3 = DetectedColor.PURPLE;
                    idx = 1;
                    break;
            }
        }
        if (gamepad2.rightBumperWasPressed()) {
            switch (idx) {
                case 1:
                    slot1 = DetectedColor.GREEN;
                    idx = 2;
                    break;
                case 2:
                    slot2 = DetectedColor.GREEN;
                    idx = 3;
                    break;
                case 3:
                    slot3 = DetectedColor.GREEN;
                    idx = 1;
                    break;
            }
        }

        // check if all slot colors are full
        if (slot1 != DetectedColor.UNKNOWN && slot2 != DetectedColor.UNKNOWN && slot3 != DetectedColor.UNKNOWN) {
            magazine.setColors(slot1, slot2, slot3);
            slot1 = slot2 = slot3 = DetectedColor.UNKNOWN;
        }
        if (gamepad2.startWasPressed() || gamepad2.backWasPressed()) {
            slot1=slot2=slot3=DetectedColor.UNKNOWN;
            magazine.setSlot(Slot.FIRST);
            idx = 1;
        }

//        if (gamepad2.xWasPressed()) launcher.setPower(launcher.getPower() == 1 ? 0 : 1);
        if (Math.abs(xPos) > Math.abs(yPos) - 150 && xPos > -100) launcherPower = 1; //TODO: make it adjustable
        else launcherPower = 0;

        if (forceLaunching) launcherPower = 1;
        launcherPower = Range.clip(launcherPower, 0, 1);
        launcherPowerVariable += (gamepad1.leftBumperWasPressed() ? -50 : 0) + (gamepad1.rightBumperWasPressed() ? 50 : 0);
        launcher.setPower(launcherPower);
        launcher.setVelocity(250 + launcherPowerVariable, AngleUnit.DEGREES);

        if (gamepad2.aWasPressed()) magazine.startLaunch(DetectedColor.GREEN);
        if (gamepad2.bWasPressed()) magazine.startLaunch(DetectedColor.PURPLE);
        if (gamepad2.yWasPressed()) magazine.autoLaunch();
        if (gamepad2.xWasPressed()) forceLaunching = !forceLaunching;

        magazine.update();

        telemetry.addData("x", odo.getPosX(DistanceUnit.MM));
        telemetry.addData("y", odo.getPosY(DistanceUnit.MM));
        telemetry.addData("h", odo.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("slot1", magazine.getLastColor(Slot.FIRST));
//        telemetry.addData("slot2", magazine.getLastColor(Slot.SECOND));
//        telemetry.addData("slot3", magazine.getLastColor(Slot.THIRD));
        telemetry.addData("intaking", intaking);
        telemetry.addData("atFullSpeed", atFullSpeed);
        telemetry.addData("launcherPower", launcherPower);
        telemetry.addData("distanceFromGoal", distanceFromGoal);

        telemetry.addData("slot1_local", slot1);
        telemetry.addData("slot2_local", slot2);
        telemetry.addData("slot3_local", slot3);
        telemetry.addData("velocity", launcher.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("intake power", intakePower);
        telemetry.addData("target", turret.getTargetDeg());
        telemetry.update();
    }
}

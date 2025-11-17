package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.Subsystems.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.constant.AllianceColor;
import org.firstinspires.ftc.teamcode.constant.Constants;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Not so Basic Drive")
@Configurable
public class testTeleop extends OpMode
{
    public turret turret;
    private AllianceColor allianceColor;
    double start; 

    public static boolean opModeIsActive = false;
    public static ElapsedTime runtime = new ElapsedTime(); 

    public static int[] mosaic = {0, 0, 0}; 
    public int mosaicPos = 0;

    double xDistanceFromGoal;
    double yDistanceFromGoal;
    double angleFromPosition;
    double angleFromHeading;

    ExecutorService magExec = Executors.newSingleThreadExecutor();
    Future<?> currentLaunch;

    @Override
    public void init()
    {
        powerMotors.init(hardwareMap); 
        colorSensor.init(hardwareMap); 
        magazine.init(hardwareMap); 

        runtime.reset();
        opModeIsActive = true;
    }

    @Override
    public void init_loop()
    {
        if (gamepad1.xWasPressed()) allianceColor = AllianceColor.BLUE;
        if (gamepad1.bWasPressed()) allianceColor = AllianceColor.BLUE;
    }

    @Override
    public void start() {
        start = runtime.now(TimeUnit.SECONDS);

        powerMotors.odo.setPosX(1767.2864, DistanceUnit.MM);
        powerMotors.odo.setPosY(451.228, DistanceUnit.MM);

        turret = new turret(hardwareMap);
        turret.zeroHere();
        turret.setLaunchAngle(45);
    }

    
    @Override
    public void loop()
    {
        powerMotors.basicDrive(
                -gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        if (runtime.now(TimeUnit.SECONDS) - start > 0.05) {
            magazine.updatePosition();
            start = runtime.now(TimeUnit.SECONDS);
        }

        xDistanceFromGoal = Constants.RED_GOAL_X - powerMotors.odo.getPosX(DistanceUnit.MM) + 150;
        yDistanceFromGoal = Constants.RED_GOAL_Y + powerMotors.odo.getPosY(DistanceUnit.MM);
        angleFromPosition = Math.atan2(yDistanceFromGoal, xDistanceFromGoal) / Math.PI * 180;
        angleFromHeading = -powerMotors.odo.getHeading(AngleUnit.DEGREES);

        if (allianceColor == AllianceColor.RED) turret.setTarget(angleFromHeading - angleFromPosition);
        if (allianceColor == AllianceColor.BLUE) turret.setTarget(angleFromHeading + angleFromPosition);


        if (gamepad1.leftBumperWasPressed()) mosaicPos = (mosaicPos - 1) % 3;
        if (gamepad1.rightBumperWasPressed()) mosaicPos = (mosaicPos + 1) % 3;
        if (gamepad1.xWasPressed())
        {
            mosaic[mosaicPos] = 2;
            mosaicPos = (mosaicPos + 1) % 3;
        }
        if (gamepad1.aWasPressed())
        {
            mosaic[mosaicPos] = 1;
            mosaicPos = (mosaicPos + 1) % 3;
        }

        if (gamepad2.yWasPressed() && (currentLaunch == null || currentLaunch.isDone())) currentLaunch = magExec.submit(magazine::autoLaunch);
        if (gamepad2.xWasPressed() && (currentLaunch == null || currentLaunch.isDone())) currentLaunch = magExec.submit(magazine::launch);

        if (gamepad1.bWasPressed()) powerMotors.toggleIntake();
        if (gamepad2.backWasPressed()) magazine.eject();

        if (gamepad2.dpadLeftWasPressed()) turret.adjustTarget(5);
        if (gamepad2.dpadRightWasPressed()) turret.adjustTarget(-5);
        if (gamepad2.dpadUpWasPressed()) turret.setLaunchAngle(turret.getLaunchAngle() + 10);
        if (gamepad2.dpadDownWasPressed()) turret.setLaunchAngle(turret.getLaunchAngle() - 10);
        turret.update();

        telemetry.addLine("Mosaic: ");
        telemetry.addData("Mosaic", toColor(mosaic[0]) + ", " + toColor(mosaic[1]) + ", " + toColor(mosaic[2]));
        telemetry.addData("Currently Editing", mosaicPos);

        telemetry.addLine(); 
        telemetry.addLine("Color Sensor:");
        telemetry.addData("Bob Detected", toColor(colorSensor.colorDetect(colorSensor.bob, 0)));
        telemetry.addData("Gary Detected", toColor(colorSensor.colorDetect(colorSensor.gary, 1)));
        telemetry.addData("Joe Detected", toColor(colorSensor.colorDetect(colorSensor.joe, 2)));
        telemetry.addData("Bob Distance (MM)", colorSensor.bob.getDistance(DistanceUnit.MM));


        telemetry.addLine(); 
        telemetry.addLine("Magazine:");
        telemetry.addData("Full", toBool(magazine.MGAr[3]));
        telemetry.addData("Magazine Contents", toColor(magazine.MGAr[0]) + ", " + toColor(magazine.MGAr[1]) + ", " + toColor(magazine.MGAr[2]));
        telemetry.addData("Active Slot", magazine.activeMG);
        telemetry.addData("Currently Active", toColor(magazine.MGAr[magazine.activeMG]));
        telemetry.addData("Top Left", toColor(magazine.MGAr[(magazine.activeMG + 2) % 3]));
        telemetry.addData("Top Right", toColor(magazine.MGAr[(magazine.activeMG + 1) % 3]));

        telemetry.addLine(); 
        telemetry.addLine("Servo Data:");
        telemetry.addData("Servo Position Variable", magazine.magazine.getPosition());
        telemetry.addData("Real Servo Position", magazine.servoPosition);

        telemetry.addLine();
        telemetry.addLine("Launcher Data");
        telemetry.addData("Velocity", powerMotors.launcher.getVelocity());

        telemetry.addLine();
        telemetry.addLine("Odometry Data");
        telemetry.addData("Heading (deg)", powerMotors.odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position (MMxMM)", powerMotors.odo.getPosition());
        telemetry.addData("X Position (MM)", powerMotors.odo.getPosX(DistanceUnit.MM));
        telemetry.addData("Y Position (MM)", powerMotors.odo.getPosY(DistanceUnit.MM));

        telemetry.update();
    }

    @Override
    public void stop()
    {
        turret.zeroHere();
        turret.update();
        opModeIsActive = false;
    }

    public String toColor(int color)
    {
        if (color == 1) return "GREEN";
        if (color == 2) return "PURPLE";
        if (color == 0) return "NOTHING";
        if (color == -1) return "UNIDENTIFIED";
        else return "WTF";
    }

    public boolean toBool(int bool)
    {
        return bool != 0;
    }
}
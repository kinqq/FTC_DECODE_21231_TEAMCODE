package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.Constants;


@TeleOp(name = "VERYBasicDrive")
@Configurable
public class testTeleop extends OpMode {

    public static ElapsedTime runtime = new ElapsedTime(); //Represents time since robot initialization

    public static int[] mosaic = {0, 0, 0}; //Array to store the expected mosaic
    public int mosaicPos = 0;//Integer to represent which index of the mosaic the user is affecting

    public double launcherSpeed = 1;
    private final AllianceColor allianceColor = AllianceColor.RED;
    double start = runtime.now(TimeUnit.SECONDS); //Represents start time

    turret turret;
    private VoltageSensor roboVoltage;

    public static boolean opModeIsActive = false;

    ExecutorService magExec = Executors.newSingleThreadExecutor();
    Future<?> currentLaunch;

    //Initialization process, runs when program initialized
    @Override
    public void init()
    {
        colorSensor.init(hardwareMap); //Initialize color sensors subsystem
        roboVoltage = hardwareMap.get(VoltageSensor.class, "Expansion Hub 1");

//      manMag.init(hardwareMap, runtime); //Initializes the manual magazine subsystem for manual magazine management
        runtime.reset();
        opModeIsActive = true;
    }

    @Override
    public void start() {
        powerMotors.init(hardwareMap); //Initialize the high power motors subsystem
        magazine.init(hardwareMap); //Initialize magazine subsystem
        powerMotors.odo.setPosX(1767.2864, DistanceUnit.MM);
        powerMotors.odo.setPosY(451.228, DistanceUnit.MM);
        turret = new turret(hardwareMap);
        turret.zeroHere();
        turret.setLaunchAngle(45);
    }

    //Main game loop, runs after play button pressed
    @Override
    public void loop() {
        //Send stick values to drive for designated drive type
        powerMotors.basicDrive(
                -gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        /*
        If player one presses left bumper cycle the affected mosaic position back one
        If player one presses right bumper cycle the affected mosaic position forward one

        If player one presses X then change the currently active mosaic piece to purple and
            automatically move mosaic position forward one
        If player one presses A then change the currently active mosaic piece to green and
            automatically move mosaic position forward one
         */
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


//        /*
//        If player two presses A shut off the intake
//        If player two presses B turn on intake
//         */
//        if (gamepad2.a) powerMotors.intakeOff(false);
//        if (gamepad2.b) powerMotors.intakeOff(true);

//        /*
//        If player 2 presses left bumper then put a green ball into launch position
//        If player 2 presses right bumper then put a purple ball into launch position
//         */
//        if (gamepad2.leftBumperWasPressed()) magazine.find(0);//Find a purple and put in launch position
//        if (gamepad2.rightBumperWasPressed()) magazine.find(1); //Find a green and put in launch position

//        if (gamepad2.rightBumperWasPressed()) magazine.servoPosition += 0.4; //Manually move servo up one slot
//        if (gamepad2.leftBumperWasPressed()) magazine.servoPosition -= 0.4; //Manually move servo down one slot

//        if (gamepad2.yWasPressed()) magazine.launch(runtime); //When player 2 presses Y activate the hammer

//        if (gamepad2.leftBumperWasPressed())
//        {
//            magazine.servoPosition = magazine.find(1);
//        }
//        if (gamepad2.rightBumperWasPressed())
//        {
//            magazine.servoPosition = magazine.find(0);
//        }

        if (gamepad2.yWasPressed()) {
            if (currentLaunch == null || currentLaunch.isDone()) {
                currentLaunch = magExec.submit(magazine::autoLaunch);
            }
        }
        if (gamepad2.xWasPressed()) {
            if (currentLaunch == null || currentLaunch.isDone()) {
                currentLaunch = magExec.submit(magazine::launch);
            }
        }

        if (gamepad1.right_trigger > 0.00) launcherSpeed = 0.5;
        if (gamepad1.left_trigger > 0.00) launcherSpeed = 1;
        if (gamepad1.yWasPressed()) magazine.MGAr[magazine.activeMG] = 1;
        //if (gamepad1.bWasPressed()) magazine.MGAr[magazine.activeMG] = 2;

        if (gamepad2.backWasPressed()) magazine.eject();

        if (gamepad1.bWasPressed()) powerMotors.toggleIntake();


        if (runtime.now(TimeUnit.SECONDS) - start > 0.05) {
            magazine.updatePosition();
            start = runtime.now(TimeUnit.SECONDS);
        }


        double xDistanceFromGoal = Constants.RED_GOAL_X - powerMotors.odo.getPosX(DistanceUnit.MM) + 150;
        double yDistanceFromGoal = Constants.RED_GOAL_Y + powerMotors.odo.getPosY(DistanceUnit.MM);
        double angleFromPosition = Math.atan2(yDistanceFromGoal, xDistanceFromGoal) / Math.PI * 180;
        double angleFromHeading = -powerMotors.odo.getHeading(AngleUnit.DEGREES);
        if (allianceColor == AllianceColor.RED) turret.setTarget(angleFromHeading - angleFromPosition);
        if (allianceColor == AllianceColor.BLUE) turret.setTarget(angleFromHeading + angleFromPosition);

        if (gamepad2.dpadLeftWasPressed()) turret.adjustTarget(-1);
        if (gamepad2.dpadRightWasPressed()) turret.adjustTarget(1);
        if (gamepad2.dpadUpWasPressed()) turret.setLaunchAngle(turret.getLaunchAngle() + 10);
        if (gamepad2.dpadDownWasPressed()) turret.setLaunchAngle(turret.getLaunchAngle() - 10);
        turret.update();

        //Report the contents of the mosaic and active mosaic position
        telemetry.addLine("Mosaic: ");
        telemetry.addData("Mosaic", mosaic[0] + ", " + mosaic[1] + ", " + mosaic[2]);
        telemetry.addData("Active Mosaic", mosaicPos);

        telemetry.addLine(); //Empty line
        telemetry.addLine("Color Sensor:"); //Caption for the color sensor section
        //Print the color currently in the active position (0: empty, 1: green, 2: purple)
        telemetry.addData("Detected Color bob", colorSensor.colorDetect(colorSensor.bob, 0));
        telemetry.addData("Detected Color gary", colorSensor.colorDetect(colorSensor.gary, 1));
        telemetry.addData("Detected Color joe", colorSensor.colorDetect(colorSensor.joe, 2));



        telemetry.addLine(); //Empty Line
        telemetry.addLine("Magazine: "); //Caption indicating the magazine section
        telemetry.addData("Is full", magazine.MGAr[3]); //Prints whether or not the magazine is full
        telemetry.addData("Active MG", magazine.MGAr[0] + ", " + magazine.MGAr[1] + ", " + magazine.MGAr[2]);
        telemetry.addData("MGAr.activeMG", magazine.MGAr[magazine.activeMG]);
        telemetry.addData("Currently active", magazine.activeMG); //Prints which slot is currently active
        telemetry.addData("Active Slot", magazine.MGAr[magazine.activeMG]); //Prints the color in the active slot
        telemetry.addData("Top Left", magazine.MGAr[(magazine.activeMG + 2) % 3]); //Prints color in top left
        telemetry.addData("Top Right", magazine.MGAr[(magazine.activeMG + 1) % 3]); //Prints color in top right

        telemetry.addLine(); //Empty Line
        telemetry.addLine("Servo Data: "); //Caption for servo data section
        telemetry.addData("Servo Position (real)", magazine.magazine.getPosition()); //Real servo position
        telemetry.addData("Servo Position (reported)", magazine.servoPosition); //Expected servo position

        telemetry.addData("Color sensor H", colorSensor.H);
        telemetry.addData("Color sensor S", colorSensor.S);
        telemetry.addData("Color sensor V", colorSensor.V);
        telemetry.addData("Distance", colorSensor.bob.getDistance(DistanceUnit.MM));
        telemetry.addData("V", powerMotors.launcher.getVelocity());


        telemetry.addData("Voltage", roboVoltage.getVoltage());
        telemetry.addLine();


        telemetry.addData("HEADING", powerMotors.odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("POS X", powerMotors.odo.getPosX(DistanceUnit.MM));
        telemetry.addData("POS Y", powerMotors.odo.getPosY(DistanceUnit.MM));

        telemetry.update();
    }

    @Override
    public void stop()
    {
        turret.zeroHere();
        turret.update();
        opModeIsActive = false;
    }
}
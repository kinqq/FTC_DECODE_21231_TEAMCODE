package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.Subsystems.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//TODO: make it check color only when magazine moves to avoid overload without timer

@TeleOp(name = "VERYBasicDrive")
@Configurable
public class testTeleop extends OpMode {

    public static ElapsedTime runtime = new ElapsedTime(); //Represents time since robot initialization
    public int[] mosaic = {0, 0, 0}; //Array to store the expected mosaic
    public int mosaicPos = 0; //Integer to represent which index of the mosaic the user is affecting

    //Initialization process, runs when program initialized
    @Override
    public void init()
    {
        magazine.init(hardwareMap); //Initialize magazine subsystem
//        manMag.init(hardwareMap, runtime); //Initializes the manual magazine subsystem for manual magazine management
        runtime.reset();
    }

    @Override
    public void start() {
        powerMotors.init(hardwareMap); //Initialize the high power motors subsystem
        colorSensor.init(hardwareMap); //Initialize color sensors subsystem
    }

    //Main game loop, runs after play button pressed
    @Override
    public void loop() {
        //Send stick values to drive for designated drive type
        powerMotors.odoDrive(
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
        if (gamepad1.leftBumperWasPressed()) mosaicPos -= 1 % 3;
        if (gamepad1.rightBumperWasPressed()) mosaicPos += 1 % 3;
        if (gamepad1.xWasPressed()) mosaic[mosaicPos] = 2;
        if (gamepad1.aWasPressed()) mosaic[mosaicPos] = 1;


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


        if (gamepad2.yWasPressed()) magazine.launch(runtime); //When player 2 presses Y activate the hammer

        //TODO: UPDATE THIS TO USE AN AUTOLAUNCH FUNCTION
        if (magazine.MGAr[3] == 1)
        {
            if (mosaic[0] == 1) magazine.find(1);
            if (mosaic[0] == 2) magazine.find(0);
        }

        magazine.updatePosition();


        //Report the contents of the mosaic and active mosaic position
        telemetry.addLine("Mosaic: ");
        telemetry.addData("Mosaic", mosaic[0] + ", " + mosaic[1] + ", " + mosaic[2]);
        telemetry.addData("Active Mosaic", mosaicPos);

        telemetry.addLine(); //Empty line
        telemetry.addLine("Color Sensor:"); //Caption for the color sensor section
        //Print the color currently in the active position (0: empty, 1: green, 2: purple)
        telemetry.addData("Detected Color", colorSensor.colorDetect(colorSensor.bob));

        telemetry.addLine(); //Empty Line
        telemetry.addLine("Magazine: "); //Caption indicating the magazine section
        telemetry.addData("Is full", magazine.MGAr[3]); //Prints whether or not the magazine is full
        telemetry.addData("Currently active", magazine.activeMG); //Prints which slot is currently active
        telemetry.addData("Active Slot", magazine.MGAr[magazine.activeMG]); //Prints the color in the active slot
        telemetry.addData("Top Right", magazine.MGAr[(magazine.activeMG + 1) % 3]); //Prints color in top right
        telemetry.addData("Top Left", magazine.MGAr[(magazine.activeMG + 2) % 3]); //Prints color in top left

        telemetry.addLine(); //Empty Line
        telemetry.addLine("Servo Data: "); //Caption for servo data section
        telemetry.addData("Servo Position (real)", magazine.magazine.getPosition()); //Real servo position
        telemetry.addData("Servo Position (reported)", magazine.servoPosition); //Expected servo position
    }
}
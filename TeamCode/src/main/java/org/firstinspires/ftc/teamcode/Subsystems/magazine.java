package org.firstinspires.ftc.teamcode.Subsystems;

import java.util.concurrent.locks.ReentrantLock;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.testTeleop;

import java.util.concurrent.TimeUnit;

public class magazine
{
    public static Servo magazine; //Servo that controls the position of the magazine
    public static Servo hammer; //Servo that controls the position of the hammer
    
    public static double servoPosition = 0; //Variable that represents the expected servo position
    //Array to store the magazine contents based on the initial position (0: active slot, 1: top right, 2: top left, 3: check for full)
    public static int[] MGAr = {0, 0, 0, 0};
    public static int activeMG = 0; //Stores what index of MGAr is currently active
    public static boolean readyToLaunch; //Stores whether or not the robot is ready to launch
    public static boolean launching;
    public static boolean autoLaunching;

    public static ElapsedTime stuckTimer = new ElapsedTime();

    private static final ReentrantLock opLock = new ReentrantLock(true);


    //Initialization, runs when called in teleop
    public static void init(HardwareMap hwMap)
    {
        magazine = hwMap.get(Servo.class, "MG"); //Initializes the magazine based on the servo "MG"
        hammer = hwMap.get(Servo.class, "HA"); //Initializes the hammer based on the servo "HA"

        //Reset variables
        servoPosition = 0;
        activeMG = 0;
        MGAr[0] = 0;
        MGAr[1] = 0;
        MGAr[2] = 0;
        MGAr[3] = 0;
        readyToLaunch = false;
        launching = false;
        autoLaunching = false;

        hammer.setPosition(0.48); //Move the hammer into initialized position
        magazine.setPosition(servoPosition); //Move magazine to initialized position

        stuckTimer.reset();
    }

    //Finds a ball of specified color (0: purple, 1: green, 2: empty)
    public static double find(int Green_Purple_Empty)
    {
        switch (Green_Purple_Empty) //Checks cases based on purple_green_empty
        {
            case 0: //If purple
                //All same code but checking for green
                //powerMotors.intakeOff(true);
                if (MGAr[0] == 1)
                {
                    readyToLaunch = true;
                    return 0.55;
                }
                else if (MGAr[1] == 1)
                {
                    readyToLaunch = true;
                    return 0.95;
                }
                else if (MGAr[2] == 1)
                {
                    readyToLaunch = true;
                    return 0.2;
                }
                break;
            case 1:
                //powerMotors.intakeOff(true); //Disables intake

                if (MGAr[0] == 2) //If slot #1 from init has a purple
                {
                    readyToLaunch = true; //Set readyToLaunch to true
                    return 0.55; //Set servo to the slot #1 intake position
                }
                else if (MGAr[1] == 2) //If slot #2 from init has purple
                {
                    readyToLaunch = true; //Set readyToLaunch to true
                    return 0.95; //Set servo to the slot #2 intake position
                }
                else if (MGAr[2] == 2) //If slot #3 from init has purple
                {
                    readyToLaunch = true; //Set readyToLaunch to true
                    return 0.2; //Set servo to the slot #3 intake position
                }
                break;
            case 2:
                powerMotors.intakeOff(false); //Turn intake off

                if (MGAr[0] == 0) return 0; //Set servo to init position 1
                else if (MGAr[1] == 0) return 0.4; //Set servo to init position 2
                else if (MGAr[2] == 0) return 0.8; //Set servo to init position 3
                break;
            default:
                return servoPosition;
        }
        return 0;
    }

    //Check the color sensors and put that color in the correct position within MGAr
    public static void checkColors()
    {
        MGAr[activeMG] = colorSensor.colorDetect(colorSensor.bob); //Check bob and set the return to the active slot
    }

    //Manual launch program
    public static void launch(ElapsedTime runtime)
    {
        opLock.lock();
        try
        {
            launching = true; //Set launching to true preventing magazine movement
            hammer.setPosition(0.7); //Move the hammer to fully pushed position

            double start = runtime.now(TimeUnit.SECONDS); //Sets the variable start to the time when function was called
            while (runtime.now(TimeUnit.SECONDS) - start < 0.1) ; //Wait 0.5 seconds

            hammer.setPosition(0.48); //Set hammer to waiting position

            start = runtime.now(TimeUnit.SECONDS); //Sets the variable start to the time when function was called
            while (runtime.now(TimeUnit.SECONDS) - start < 0.1) ; //Wait 0.5 seconds

            //launching = false; //Set launching to false allowing movement
            //readyToLaunch = false; //Set readyToLaunch to false stopping another launch
        } finally
        {
            opLock.unlock();
        }
    }

    public static void autoLaunch()
    {
        opLock.lock();
        try
        {
            double specialServoPosition = 0;
            hammer.setPosition(0.48);

            specialServoPosition = find(testTeleop.mosaic[0] - 1);
            magazine.setPosition(specialServoPosition);
            waitSeconds(0.5);

            hammer.setPosition(0.7);
            waitSeconds(0.2);
            hammer.setPosition(0.48);
            if (specialServoPosition == 0.55) MGAr[0] = 0;
            if (specialServoPosition == 0.95) MGAr[1] = 0;
            if (specialServoPosition == 2) MGAr[2] = 0;

            waitSeconds(0.5);
            specialServoPosition = find(testTeleop.mosaic[1] - 1);
            magazine.setPosition(specialServoPosition);
            waitSeconds(0.5);

            hammer.setPosition(0.7);
            waitSeconds(0.2);
            hammer.setPosition(0.48);
            if (specialServoPosition == 0.55) MGAr[0] = 0;
            if (specialServoPosition == 0.95) MGAr[1] = 0;
            if (specialServoPosition == 2) MGAr[2] = 0;

            waitSeconds(0.5);
            specialServoPosition = find(testTeleop.mosaic[2] - 1);
            magazine.setPosition(specialServoPosition);
            waitSeconds(0.5);

            hammer.setPosition(0.7);
            waitSeconds(0.2);
            hammer.setPosition(0.48);
            if (specialServoPosition == 0.55) MGAr[0] = 0;
            if (specialServoPosition == 0.95) MGAr[1] = 0;
            if (specialServoPosition == 2) MGAr[2] = 0;

            waitSeconds(0.5);
            magazine.setPosition(0);
            MGAr = new int[] {0, 0, 0, 0};
            activeMG = 0;
            servoPosition = 0;
        } finally {
            opLock.unlock();
        }
    }

    //Update the physical servo position, done at end to ensure time for all math to happen making only one move happen
    public static void updatePosition()
    {
        if (!opLock.isLocked())
        {
            if (MGAr[3] == 0) servoPosition = find(2);

            if (servoPosition == 0)
                activeMG = 0; //If servo is in position one then activeMG equals zero
            else if (servoPosition == 0.4)
                activeMG = 1; //If servo is in position two then activeMG equals 1
            else if (servoPosition == 0.8)
                activeMG = 2; //If servo is in position three then activeMG equals 2

            if (MGAr[0] != 0 && MGAr[1] != 0 && MGAr[2] != 0)
                MGAr[3] = 1; //If the magazine is full update the full check slot
            else MGAr[3] = 0; //If magazine is not full set full check to 0

            if (MGAr[3] == 0) magazine.setPosition(servoPosition); //Update the physical servo position
        }
    }

    public static void reCheck()
    {
        servoPosition = 0;
        magazine.setPosition(servoPosition);
        checkColors();
        waitSeconds(0.5);
        servoPosition = 0.4;
        magazine.setPosition(servoPosition);
        checkColors();
        waitSeconds(0.5);
        servoPosition = 0.8;
        magazine.setPosition(servoPosition);
        checkColors();
        waitSeconds(0.5);
    }

    private static void waitSeconds(double seconds) {
        try {
            Thread.sleep((long)(seconds * 1000));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

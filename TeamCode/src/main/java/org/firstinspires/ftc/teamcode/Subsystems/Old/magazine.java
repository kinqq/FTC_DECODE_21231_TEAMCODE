package org.firstinspires.ftc.teamcode.Subsystems.Old;

import java.util.concurrent.locks.ReentrantLock;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.testTeleop;

@Configurable
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
        magazine = hwMap.get(Servo.class, "turntable"); //Initializes the magazine based on the servo "MG"
        hammer = hwMap.get(Servo.class, "hammer"); //Initializes the hammer based on the servo "HA"

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
                    return 0.6;
                }
                else if (MGAr[1] == 1)
                {
                    readyToLaunch = true;
                    return 0.98;
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
                    return 0.6; //Set servo to the slot #1 intake position
                }
                else if (MGAr[1] == 2) //If slot #2 from init has purple
                {
                    readyToLaunch = true; //Set readyToLaunch to true
                    return 0.98; //Set servo to the slot #2 intake position
                }
                else if (MGAr[2] == 2) //If slot #3 from init has purple
                {
                    readyToLaunch = true; //Set readyToLaunch to true
                    return 0.2; //Set servo to the slot #3 intake position
                }
                break;
            case 2:
                //powerMotors.intakeOff(false); //Turn intake off
                if (MGAr[0] == 0) return 0; //Set servo to init position 1
                else if (MGAr[1] == 0) return 0.4; //Set servo to init position 2
                else if (MGAr[2] == 0) return 0.8; //Set servo to init position 3
                break;
            default:
                return servoPosition;
        }
        return servoPosition;
    }

    //Check the color sensors and put that color in the correct position within MGAr
    public static void checkSlots()
    {
        MGAr[activeMG] = colorSensor.colorDetect(colorSensor.bob, 0);
        MGAr[(activeMG + 2) % 3] = colorSensor.colorDetect(colorSensor.gary, 1); //Check bob and set the return to the active slot
        MGAr[(activeMG + 1) % 3] = colorSensor.colorDetect(colorSensor.joe, 2); //Check bob and set the return to the active slot

        /*
        if (colorSensor.gary.getDistance(DistanceUnit.MM) < 65) MGAr[(activeMG + 2) % 3] = -1;
        else MGAr[(activeMG + 2) % 3] = 0;
        if (colorSensor.joe.getDistance(DistanceUnit.MM) < 65) MGAr[(activeMG + 1) % 3] = -1;
        else MGAr[(activeMG + 1) % 3] = 0;

         */

    }

    public static void checkColors()
    {
        MGAr[activeMG] = colorSensor.colorDetect(colorSensor.bob, 0);
        MGAr[(activeMG + 2) % 3] = colorSensor.colorDetect(colorSensor.gary, 1); //Check bob and set the return to the active slot
        MGAr[(activeMG + 1) % 3] = colorSensor.colorDetect(colorSensor.joe, 2); //Check bob and set the return to the active slot
    }

    public static void launch()
    {
        opLock.lock();
        try
        {
            double specialServoPosition = 0.2;

            //powerMotors.launcher.setPower(1);
            powerMotors.launcher.setVelocity(1850);
            hammer.setPosition(0.48);

            for (int i = 0; i < 3; i++)
            {
                magazine.setPosition(specialServoPosition);

                //waitSeconds((0.05 + (3.0 - 0.05) * Math.pow(1.0 - (powerMotors.launcher.getVelocity() / 1900), 2.0) / (roboVoltage.getVoltage() / 12.5)) + 0.3);
                while (powerMotors.launcher.getVelocity() < 1850);

                waitSeconds(0.2);

                hammer.setPosition(0.7);
                waitSeconds(0.075);
                hammer.setPosition(0.48);

                specialServoPosition += 0.4;
                waitSeconds(0.5);
            }

            magazine.setPosition(0);
            MGAr = new int[] {0, 0, 0, 0};
            activeMG = 0;
            servoPosition = 0;
            powerMotors.launcher.setPower(0);

        } finally {
            opLock.unlock();
        }
    }

    public static void autoLaunch()
    {
        opLock.lock();
        try
        {
            double specialServoPosition = 0;

            powerMotors.launcher.setPower(1);
            hammer.setPosition(0.48);
            magazine.setPosition(0);

            checkColors();
            waitSeconds(0.01);



            for (int i = 0; i < 3; i++)
            {
                specialServoPosition = find(testTeleop.mosaic[i] - 1);
                magazine.setPosition(specialServoPosition);

                while (powerMotors.launcher.getVelocity() < 1850);

                hammer.setPosition(0.7);
                waitSeconds(0.057);
                hammer.setPosition(0.48);

                if (specialServoPosition == 0.6) MGAr[0] = 0;
                if (specialServoPosition == 0.98) MGAr[1] = 0;
                if (specialServoPosition == 0.2) MGAr[2] = 0;
                waitSeconds(0.5);
            }

            magazine.setPosition(0);
            MGAr = new int[] {0, 0, 0, 0};
            activeMG = 0;
            servoPosition = 0;
            powerMotors.launcher.setPower(0);

        } finally {
            opLock.unlock();
        }
    }

    //Update the physical servo position, done at end to ensure time for all math to happen making only one move happen
    public static void updatePosition()
    {
        if (!opLock.isLocked())
        {
            checkSlots();

            servoPosition = find(2);

            if (servoPosition == 0)
                activeMG = 0; //If servo is in position one then activeMG equals zero
            else if (servoPosition == 0.4)
                activeMG = 1; //If servo is in position two then activeMG equals 1
            else if (servoPosition == 0.8)
                activeMG = 2; //If servo is in position three then activeMG equals 2

            if (MGAr[0] != 0 && MGAr[1] != 0 && MGAr[2] != 0)
                MGAr[3] = 1; //If the magazine is full update the full check slot
            else MGAr[3] = 0; //If magazine is not full set full check to 0

            hammer.setPosition(0.48);
            magazine.setPosition(servoPosition); //Update the physical servo position
        }
    }

    public static void eject()
    {
        powerMotors.intake.setDirection(DcMotorSimple.Direction.FORWARD);
        powerMotors.intakeOff(false);
        waitSeconds(0.5);
        powerMotors.intakeOff(true);
        powerMotors.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        powerMotors.intakeOff(false);
    }

    private static void waitSeconds(double seconds) {
        try {
            Thread.sleep((long)(seconds * 1000));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

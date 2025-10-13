package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class magazine
{
    public static Servo magazine;

    public static RevColorSensorV3 bob; //Intake Slot
    public static RevColorSensorV3 gary; //Top Left
    public static RevColorSensorV3 joe; //Top right
    
    public static double servoPosition = 0.1;
    public static int[] MGAr = {0, 0, 0, 0};
    public static int activeMG = 0;


    public static void init(HardwareMap hwMap)
    {
        magazine = hwMap.get(Servo.class, "MG");

        bob = hwMap.get(RevColorSensorV3.class, "bob");
        gary = hwMap.get(RevColorSensorV3.class, "gary");
        joe = hwMap.get(RevColorSensorV3.class, "joe");


        servoPosition = 0.1;
        activeMG = 0;
        MGAr[0] = 0;
        MGAr[1] = 0;
        MGAr[2] = 0;

        driveMotors.intakeOff(false);

        magazine.setPosition(servoPosition);
    }

    public static void find(int Purple_Green_Empty)
    {
        if (Purple_Green_Empty == 0 && MGAr[activeMG] != 2) {
            driveMotors.intakeOff(true);

            if (MGAr[0] == 2) servoPosition = 0.1;
            else if (MGAr[1] == 2) servoPosition = 0.5;
            else if (MGAr[2] == 2) servoPosition = 0.9;
        } else if (Purple_Green_Empty == 1 && MGAr[activeMG] != 1) {
            driveMotors.intakeOff(true);

            if (MGAr[0] == 1) servoPosition = 0.1;
            else if (MGAr[1] == 1) servoPosition = 0.5;
            else if (MGAr[2] == 1) servoPosition = 0.9;

        } else if (Purple_Green_Empty == 2 && MGAr[activeMG] != 0) {
            driveMotors.intakeOff(false);

            if (MGAr[0] == 0) servoPosition = 0.1;
            else if (MGAr[1] == 0) servoPosition = 0.5;
            else if (MGAr[2] == 0) servoPosition = 0.9;
        }
    }

    public static void checkColors()
    {
        if (bob.green() > 80 && bob.green() > bob.red() && bob.green() > bob.blue()) MGAr[activeMG] = 1;
        if (
                bob.green() > 75 && bob.green() < 80 &&
                bob.blue() > 73 && bob.blue() < 81 &&
                bob.red() > 49 && bob.red() < 55
        ) MGAr[activeMG] = 2;
        if (
                bob.green() > 70 && bob.green() < 75 &&
                bob.blue() > 60 && bob.blue() < 65 &&
                bob.red() > 40 && bob.red() < 45
        ) MGAr[activeMG] = 0;


        if (gary.green() > 75 && gary.green() > gary.red() && gary.green() > gary.blue()) MGAr[(activeMG + 2) % 3] = 1;
        else if (
                gary.green() > 80 && gary.green() < 90 &&
                gary.blue() > 80 && gary.blue() < 90 &&
                gary.red() > 50 && gary.red() < 60
        ) MGAr[(activeMG + 2) % 3] = 2;
        else if (
                gary.green() > 75 && gary.green() < 85 &&
                gary.blue() > 65 && gary.blue() < 70 &&
                gary.red() > 45 && gary.red() < 50
        ) MGAr[(activeMG + 2) % 3] = 0;

        if (joe.green() > 88 && joe.blue() < 78 && joe.red() < 45) MGAr[(activeMG + 1) % 3] = 1;
        else if (
                joe.green() > 80 && joe.green() < 90 &&
                joe.blue() > 80 && joe.blue() < 90 &&
                joe.red() > 50 && joe.red() < 60
        ) MGAr[(activeMG + 1) % 3] = 2;
        else if (
                joe.green() > 75 && joe.green() < 85 &&
                joe.blue() > 65 && joe.blue() < 70 &&
                joe.red() > 45 && joe.red() < 50
        ) MGAr[(activeMG + 1) % 3] = 0;

    }

    public static void updatePosition()
    {
        if (driveMotors.intakeRunning) find(2);

        if (servoPosition > 0.91) servoPosition = 0.1;
        if (servoPosition < 0.09) servoPosition = 0.9;

        if (servoPosition == 0.1) activeMG = 0;
        else if (servoPosition == 0.5) activeMG = 1;
        else if (servoPosition == 0.9) activeMG = 2;

        magazine.setPosition(servoPosition);
    }
}

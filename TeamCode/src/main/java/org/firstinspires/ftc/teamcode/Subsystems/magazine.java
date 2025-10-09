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

        magazine.setPosition(servoPosition);
    }

    public static void find(int Purple_Green_Empty)
    {
        if (Purple_Green_Empty == 0 && MGAr[activeMG] != 2) {
            drive.intakeOff(true);

            if (MGAr[0] == 2) servoPosition = 0.1;
            else if (MGAr[1] == 2) servoPosition = 0.5;
            else if (MGAr[2] == 2) servoPosition = 0.9;
        } else if (Purple_Green_Empty == 1 && MGAr[activeMG] != 1) {
            drive.intakeOff(true);

            if (MGAr[0] == 1) servoPosition = 0.1;
            else if (MGAr[1] == 1) servoPosition = 0.5;
            else if (MGAr[2] == 1) servoPosition = 0.9;

        } else if (Purple_Green_Empty == 2 && MGAr[activeMG] != 0) {
            drive.intakeOff(false);

            if (MGAr[0] == 0) servoPosition = 0.1;
            else if (MGAr[1] == 0) servoPosition = 0.5;
            else if (MGAr[2] == 0) servoPosition = 0.9;
        }
    }

    public static void checkColors()
    {
        if (bob.green() > 130 && bob.blue() < 130 && bob.red() < 130) MGAr[activeMG] = 1;
        else if (bob.green() > 100 && bob.blue() > 130 && bob.red() > 100) MGAr[activeMG] = 2;

        if (gary.green() > 130 && gary.blue() < 130 && gary.red() < 130) MGAr[(activeMG + 2) % 3] = 1;
        else if (gary.green() > 100 && gary.blue() > 130 && gary.red() > 100) MGAr[(activeMG + 2) % 3] = 2;

        if (joe.green() > 130 && joe.blue() < 130 && joe.red() < 130) MGAr[(activeMG + 1) % 3] = 1;
        else if (joe.green() > 100 && joe.blue() > 130 && joe.red() > 100) MGAr[(activeMG + 1) % 3] = 2;
    }

    public static void updatePosition()
    {
        if (drive.intakeRunning) find(2);
        else find(0);

        if (servoPosition > 0.91) servoPosition = 0.1;
        if (servoPosition < 0.09) servoPosition = 0.9;

        if (servoPosition == 0.1) activeMG = 0;
        else if (servoPosition == 0.5) activeMG = 1;
        else if (servoPosition == 0.9) activeMG = 2;

        magazine.setPosition(servoPosition);
    }
}

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

        servoPosition = 0;
        activeMG = 0;
        MGAr[0] = 0;
        MGAr[1] = 0;
        MGAr[2] = 0;

        magazine.setPosition(servoPosition);
    }

    public static void find(int Purple_Green_Empty)
    {
        if (Purple_Green_Empty == 0 && MGAr[activeMG] != 2) {
            powerMoters.intakeOff(true);

            if (MGAr[0] == 2) servoPosition = 0.8;
            else if (MGAr[1] == 2) servoPosition = 0.4;
            else if (MGAr[2] == 2) servoPosition = 1;
        } else if (Purple_Green_Empty == 1 && MGAr[activeMG] != 1) {
            powerMoters.intakeOff(true);

            if (MGAr[0] == 1) servoPosition = 0.8;
            else if (MGAr[1] == 1) servoPosition = 0.4;
            else if (MGAr[2] == 1) servoPosition = 1;

        } else if (Purple_Green_Empty == 2 && MGAr[activeMG] != 0) {
            powerMoters.intakeOff(false);

            if (MGAr[0] == 0) servoPosition = 0;
            else if (MGAr[1] == 0) servoPosition = 0.4;
            else if (MGAr[2] == 0) servoPosition = 0.8;
        }
    }

    public static void checkColors()
    {
        MGAr[activeMG] = colorSensor.colorDetect(colorSensor.bob);
    }

    public static void updatePosition()
    {
        if (powerMoters.intakeRunning) find(2);

        if (servoPosition == 0) activeMG = 0;
        else if (servoPosition == 0.4) activeMG = 1;
        else if (servoPosition == 0.8) activeMG = 2;

        if (servoPosition == 0.6) activeMG = 3;
        else if (servoPosition == 0.2) activeMG = 4;
        else if (servoPosition == 1) activeMG = 5;

        magazine.setPosition(servoPosition);
    }
}

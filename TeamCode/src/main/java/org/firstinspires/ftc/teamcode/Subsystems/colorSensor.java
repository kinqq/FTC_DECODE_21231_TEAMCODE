package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class colorSensor
{
    public enum DetectedColor
    {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public static RevColorSensorV3 bob; //Intake Slot
    public static RevColorSensorV3 gary; //Top Left
    public static RevColorSensorV3 joe; //Top right

    public static void init(HardwareMap hwMap)
    {
        bob = hwMap.get(RevColorSensorV3.class, "bob");
        gary = hwMap.get(RevColorSensorV3.class, "gary");
        joe = hwMap.get(RevColorSensorV3.class, "joe");

        bob.setGain(100);
    }

    public static DetectedColor bobDetect(Telemetry telemetry)
    {
        NormalizedRGBA normRGBA = bob.getNormalizedColors();
        
        float normRed, normGreen, normBlue;
        normRed = normRGBA.red / (float) bob.getRawLightDetected();
        normGreen = normRGBA.green / (float) bob.getRawLightDetected();
        normBlue = normRGBA.blue / (float) bob.getRawLightDetected();

        double redToGreen, greenToGreen, blueToGreen;
        redToGreen = 0.0005;
        greenToGreen = 0.0013;
        blueToGreen = 0.001;

        double redToPurple, greenToPurple, blueToPurple;
        redToPurple = 5;
        greenToPurple = 5;
        blueToPurple = 5;

        telemetry.addData("Red", normRed);
        telemetry.addData("Green", normGreen);
        telemetry.addData("Blue", normBlue);

        if (normRed <= redToGreen && normGreen <= greenToGreen && normBlue <= blueToGreen) return DetectedColor.GREEN;
        else if (normRed >= redToPurple && normGreen >= greenToPurple && normBlue >= blueToPurple) return DetectedColor.PURPLE;
        else return DetectedColor.UNKNOWN;

    }

}

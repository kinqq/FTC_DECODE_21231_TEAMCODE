package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class colorSensor
{
    public static RevColorSensorV3 bob; //Intake Slot color sensor
    public static RevColorSensorV3 gary; //Top Left color sensor
    public static RevColorSensorV3 joe; //Top right color sensor

    //Initialize the color sensors
    public static void init(HardwareMap hwMap)
    {
        bob = hwMap.get(RevColorSensorV3.class, "bob"); //Set bob to "bob"
        gary = hwMap.get(RevColorSensorV3.class, "gary"); //Set gary to "gary"
        joe = hwMap.get(RevColorSensorV3.class, "joe"); //Set joe to "joe"

       // bob.setGain(500); //Set the gain of bob to 500 for more accurate testing
    }

    public static float H;
    public static float S;
    public static float V;

    public static int colorDetect(RevColorSensorV3 color, int bob_gary_joe)
    {
        NormalizedRGBA rgba = color.getNormalizedColors(); //Set normalized RGBA to the normalized values of the indicated color sensor


        //Some weird math to normalize the RGBA even further
        float r = clamp01(rgba.red), g = clamp01(rgba.green), b = clamp01(rgba.blue);
        float sum = r + g + b; if (sum < 1e-6f) sum = 1e-6f;
        float rc = r / sum, gc = g / sum, bc = b / sum;

        //Make the RGBA into HSV
        float[] hsv = new float[3];
        Color.RGBToHSV((int)(rc * 255f), (int)(gc * 255f), (int)(bc * 255f), hsv);

        //Separate the HSV values
        H = hsv[0];
        S = hsv[1];
        V = hsv[2];

        if (bob_gary_joe == 1) {
            if (H < 172 && S > 0.51) return 1; //When the conditions are met for green return 1
            else if (H > 185 && S <= 0.49)
                return 2; //When the conditions are met for purple return 2
                //else if (H < 163 && V >= 0.4) return 0; //When no conditions are met return 0
            else return 0;
        } else if (bob_gary_joe == 0) {
            if (H < 165 && S >= 0.4) return 1; //When the conditions are met for green return 1
            else if (H > 180 && S <= 0.5)
                return 2; //When the conditions are met for purple return 2
                //else if (H < 163 && V >= 0.4) return 0; //When no conditions are met return 0
            else return 0;
        } else if (bob_gary_joe == 2) {
            if (H <= 165 && S > 0.52) return 1; //When the conditions are met for green return 1
            else if (H >= 170 && S <= 0.5)
                return 2; //When the conditions are met for purple return 2
                //else if (H < 163 && V >= 0.4) return 0; //When no conditions are met return 0
            else return 0;
        } else return -2;
    }

    //More weird math I don't understand but works
    private static float clamp01(float v) {
        return v < 0f ? 0f : (v > 1f ? 1f : v);
    }
    
}

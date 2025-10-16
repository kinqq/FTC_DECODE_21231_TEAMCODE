package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.TurnTable;

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

        bob.setGain(500);
    }

    public static int colorDetect(RevColorSensorV3 color)
    {
        NormalizedRGBA rgba = color.getNormalizedColors();
        float r = clamp01(rgba.red), g = clamp01(rgba.green), b = clamp01(rgba.blue);
        // 크로마틱 정규화 → HSV
        float sum = r + g + b; if (sum < 1e-6f) sum = 1e-6f;
        float rc = r / sum, gc = g / sum, bc = b / sum;

        float[] hsv = new float[3];
        Color.RGBToHSV((int)(rc * 255f), (int)(gc * 255f), (int)(bc * 255f), hsv);

        float H = hsv[0];
        float S = hsv[1];
        float V = hsv[2];

        //if (S < 0.25f || V < 0.10f) return DetectedColor.UNKNOWN;
        if (H <= 165 && S >= 0.45) return 1;
        else if (H >= 175)  return 2;
        else return 0;

    }

    private static float clamp01(float v) {
        return v < 0f ? 0f : (v > 1f ? 1f : v);
    }
    
}

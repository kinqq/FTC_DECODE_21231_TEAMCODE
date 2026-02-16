package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constant.DetectedColor;

@TeleOp (name = "Color Test", group = "Test")
public class ColorTest extends OpMode
{
    private NormalizedColorSensor colorSensor;
    private RevColorSensorV3 rev;

    double h = 0;
    double s = 0;

    @Override
    public void init()
    {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        rev = hardwareMap.get(RevColorSensorV3.class, "color");
    }

    @Override
    public void loop()
    {

        telemetry.addData("Detected", getDetectedColor());
        telemetry.addData("H", h);
        telemetry.addData("S", s);
        telemetry.addData("D", rev.getDistance(DistanceUnit.MM));
        telemetry.update();
    }


    public DetectedColor getDetectedColor()
    {
        NormalizedRGBA color = colorSensor.getNormalizedColors();

        float sum = color.red + color.green + color.blue;
        if (sum < 1e-6f) {
            return DetectedColor.UNKNOWN;
        }

        float r = color.red / sum;
        float g = color.green / sum;
        float b = color.blue / sum;

        float max = Math.max(r, Math.max(g, b));
        float min = Math.min(r, Math.min(g, b));
        float delta = max - min;

        float s = (max <= 1e-6f) ? 0f : (delta / max);

        float h;
        if (delta <= 1e-6f) {
            h = 0f;
        } else if (max == r) {
            h = 60f * (((g - b) / delta) % 6f);
        } else if (max == g) {
            h = 60f * (((b - r) / delta) + 2f);
        } else {
            h = 60f * (((r - g) / delta) + 4f);
        }
        if (h < 0f) h += 360f;

        this.h = h;
        this.s = s;

        if (s < 0.20f) {
            return DetectedColor.UNKNOWN;
        }

        if (h <= 160f && s >= 0.5) {
            return DetectedColor.GREEN;
        } else if (h >= 154) {
            return DetectedColor.PURPLE;
        } else {
            return DetectedColor.UNKNOWN;
        }
    }
}

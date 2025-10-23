package org.firstinspires.ftc.teamcode.test;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import dev.frozenmilk.sinister.loading.Pinned;

@TeleOp(name = "TestColorSensor", group = "Test")
@Configurable
public class TestColorSensor extends OpMode {
    NormalizedColorSensor cs;

    @Override
    public void init() {
        cs = hardwareMap.get(NormalizedColorSensor.class, "color");
        cs.setGain(200.0f);

        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        NormalizedRGBA rgba = cs.getNormalizedColors();
        float r=rgba.red, g=rgba.green, b=rgba.blue;
        float[] hsv = new float[3];
        Color.colorToHSV(rgba.toColor(), hsv);
        float H=hsv[0], S=hsv[1], V=hsv[2];


        if (V < 0.23) telemetry.addLine("UNKNOWN");
        else if (173.0 < H && H < 187.0) telemetry.addLine("PURPLE");
        else if (152.0 < H && H < 164.0) telemetry.addLine("GREEN");


        telemetry.addData("HSV", "H=%.1f  S=%.2f  V=%.2f", H, S, V);
        telemetry.addData("RGB", "r=%.3f g=%.3f b=%.3f", r, g, b);
    }
}

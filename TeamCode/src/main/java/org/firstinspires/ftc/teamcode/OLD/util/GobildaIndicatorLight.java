package org.firstinspires.ftc.teamcode.OLD.util;

import com.qualcomm.robotcore.hardware.Servo;

public class GobildaIndicatorLight {

    private final Servo servo;

    private static final double MIN_POS = 0.277; // Red (1100us)
    private static final double MAX_POS = 0.722; // Violet (1900us)

    public GobildaIndicatorLight(Servo servo) {
        this.servo = servo;
    }

    public void setColorHex(String hex) {
        if (hex == null || !hex.matches("^#?[0-9a-fA-F]{6}$")) {
            throw new IllegalArgumentException("Invalid hex color: " + hex);
        }

        if (hex.startsWith("#")) {
            hex = hex.substring(1);
        }

        int r = Integer.parseInt(hex.substring(0, 2), 16);
        int g = Integer.parseInt(hex.substring(2, 4), 16);
        int b = Integer.parseInt(hex.substring(4, 6), 16);

        float[] hsv = rgbToHsv(r, g, b);
        float hue = hsv[0];

        if (hsv[1] < 0.1) {
            servo.setPosition(1.0);
            return;
        }

        hue = clamp(hue, 0f, 270f);

        double position = MIN_POS + (hue / 270.0) * (MAX_POS - MIN_POS);
        servo.setPosition(position);
    }

    public void off() {
        servo.setPosition(0.0);
    }

    public void white() {
        servo.setPosition(1.0);
    }

    private static float[] rgbToHsv(int r, int g, int b) {
        float rf = r / 255f;
        float gf = g / 255f;
        float bf = b / 255f;

        float max = Math.max(rf, Math.max(gf, bf));
        float min = Math.min(rf, Math.min(gf, bf));
        float delta = max - min;

        float h;
        if (delta == 0) {
            h = 0;
        } else if (max == rf) {
            h = 60 * (((gf - bf) / delta) % 6);
        } else if (max == gf) {
            h = 60 * (((bf - rf) / delta) + 2);
        } else {
            h = 60 * (((rf - gf) / delta) + 4);
        }

        if (h < 0) h += 360;

        float s = max == 0 ? 0 : delta / max;
        float v = max;

        return new float[]{h, s, v};
    }

    private static float clamp(float val, float min, float max) {
        return Math.max(min, Math.min(max, val));
    }
}

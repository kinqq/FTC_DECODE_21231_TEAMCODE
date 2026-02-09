package org.firstinspires.ftc.teamcode.OLD.autonomous;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.OLD.constant.AllianceColor;

public final class AllianceTransform {
    private static final double FIELD_W = 144.0;

    private final boolean mirrorToBlue;

    public AllianceTransform(AllianceColor allianceColor) {
        this.mirrorToBlue = (allianceColor == AllianceColor.BLUE);
    }

    public Pose pose(double x, double y) {
        if (!mirrorToBlue) return new Pose(x, y);
        return new Pose(FIELD_W - x, y);
    }

    public Pose pose(double x, double y, double headingRad) {
        if (!mirrorToBlue) return new Pose(x, y, wrapRad(headingRad));
        return new Pose(FIELD_W - x, y, wrapRad(Math.PI - headingRad));
    }

    public double headingRad(double headingRad) {
        if (!mirrorToBlue) return wrapRad(headingRad);
        return wrapRad(Math.PI - headingRad);
    }

    public double headingDeg(double deg) {
        return headingRad(Math.toRadians(deg));
    }

    private static double wrapRad(double r) {
        r = (r + Math.PI) % (2.0 * Math.PI);
        if (r < 0) r += 2.0 * Math.PI;
        return r - Math.PI;
    }
}

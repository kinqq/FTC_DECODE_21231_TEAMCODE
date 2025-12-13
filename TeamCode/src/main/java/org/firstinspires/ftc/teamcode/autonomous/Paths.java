package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.constant.AllianceColor;

public class Paths {
    public final PathChain Path1, Path2, Path3, Path4, Path5, Path6,
        Path7, Path8, Path9, Path10, Path11, Path12;

    public final PathChain FirstRowIntakePath;
    public final PathChain SecondRowIntakePath;

    public final Pose FirstRowBall1, FirstRowBall2, FirstRowBall3;
    public final Pose SecondRowBall1, SecondRowBall2, SecondRowBall3;

    private static Pose mirrorPoseX(Pose redPose) {
        double fieldWidth = 144.0;
        return new Pose(
            fieldWidth - redPose.getX(),
            redPose.getY(),
            Math.PI - redPose.getHeading()
        );
    }

    private static double mirrorHeading(double h) {
        return Math.PI - h;
    }

    public Paths(Follower follower, AllianceColor allianceColor) {
        boolean red = (allianceColor == AllianceColor.RED);

        Pose p1s = new Pose(116.000, 132.000);
        Pose p1e = new Pose(92.000, 86.000);
        if (!red) { p1s = mirrorPoseX(p1s); p1e = mirrorPoseX(p1e); }
        Path1 = follower
            .pathBuilder()
            .addPath(new BezierLine(p1s, p1e))
            .setLinearHeadingInterpolation(
                red ? Math.toRadians(36) : mirrorHeading(Math.toRadians(36)),
                red ? Math.toRadians(90) : mirrorHeading(Math.toRadians(90))
            )
            .build();

        Pose p2s = new Pose(92.000, 86.000);
        Pose p2e = new Pose(92.000, 85.000);
        if (!red) { p2s = mirrorPoseX(p2s); p2e = mirrorPoseX(p2e); }
        Path2 = follower
            .pathBuilder()
            .addPath(new BezierLine(p2s, p2e))
            .setLinearHeadingInterpolation(
                red ? Math.toRadians(90) : mirrorHeading(Math.toRadians(90)),
                red ? Math.toRadians(0)  : mirrorHeading(Math.toRadians(0))
            )
            .build();

        Pose p3s = new Pose(92.000, 85.000);
        Pose p3e = new Pose(105.000, 85.000);
        if (!red) { p3s = mirrorPoseX(p3s); p3e = mirrorPoseX(p3e); }
        Path3 = follower
            .pathBuilder()
            .addPath(new BezierLine(p3s, p3e))
            .setTangentHeadingInterpolation()
            .build();

        Pose p4s = new Pose(105.000, 85.000);
        Pose p4e = new Pose(110.000, 85.000);
        if (!red) { p4s = mirrorPoseX(p4s); p4e = mirrorPoseX(p4e); }
        Path4 = follower
            .pathBuilder()
            .addPath(new BezierLine(p4s, p4e))
            .setTangentHeadingInterpolation()
            .build();

        Pose p5s = new Pose(110.000, 85.000);
        Pose p5e = new Pose(120.000, 85.000);
        if (!red) { p5s = mirrorPoseX(p5s); p5e = mirrorPoseX(p5e); }
        Path5 = follower
            .pathBuilder()
            .addPath(new BezierLine(p5s, p5e))
            .setTangentHeadingInterpolation()
            .build();

        FirstRowBall1 = new Pose(106, 85);
        FirstRowBall2 = new Pose(110, 85);
        FirstRowBall3 = new Pose(120, 85);

        FirstRowIntakePath = follower
            .pathBuilder()
            .addPath(new BezierLine(p3s, p3e))
            .addPath(new BezierLine(p4s, p4e))
            .addPath(new BezierLine(p5s, p5e))
            .setTangentHeadingInterpolation()
            .build();

        Pose p6s = new Pose(120.000, 85.000);
        Pose p6e = new Pose(94.000, 92.000);
        if (!red) { p6s = mirrorPoseX(p6s); p6e = mirrorPoseX(p6e); }
        Path6 = follower
            .pathBuilder()
            .addPath(new BezierLine(p6s, p6e))
            .setLinearHeadingInterpolation(
                red ? Math.toRadians(0)  : mirrorHeading(Math.toRadians(0)),
                red ? Math.toRadians(45) : mirrorHeading(Math.toRadians(45))
            )
            .build();

        Pose p7s = new Pose(94.000, 92.000);
        Pose p7e = new Pose(92.000, 61.000);
        if (!red) { p7s = mirrorPoseX(p7s); p7e = mirrorPoseX(p7e); }
        Path7 = follower
            .pathBuilder()
            .addPath(new BezierLine(p7s, p7e))
            .setLinearHeadingInterpolation(
                red ? Math.toRadians(45) : mirrorHeading(Math.toRadians(45)),
                red ? Math.toRadians(0) : mirrorHeading(Math.toRadians(0))
            )
            .build();

        Pose p8s = new Pose(92.000, 61.000);
        Pose p8e = new Pose(104.000, 61.000);
        if (!red) { p8s = mirrorPoseX(p8s); p8e = mirrorPoseX(p8e); }
        Path8 = follower
            .pathBuilder()
            .addPath(new BezierLine(p8s, p8e))
            .setTangentHeadingInterpolation()
            .build();

        Pose p9s = new Pose(104.000, 61.000);
        Pose p9e = new Pose(110.000, 61.000);
        if (!red) { p9s = mirrorPoseX(p9s); p9e = mirrorPoseX(p9e); }
        Path9 = follower
            .pathBuilder()
            .addPath(new BezierLine(p9s, p9e))
            .setTangentHeadingInterpolation()
            .build();

        Pose p10s = new Pose(110.000, 61.000);
        Pose p10e = new Pose(120, 61.000);
        if (!red) { p10s = mirrorPoseX(p10s); p10e = mirrorPoseX(p10e); }
        Path10 = follower
            .pathBuilder()
            .addPath(new BezierLine(p10s, p10e))
            .setTangentHeadingInterpolation()
            .build();

        SecondRowBall1 = new Pose(108, 61);
        SecondRowBall2 = new Pose(112, 61);
        SecondRowBall3 = new Pose(120, 61);

        SecondRowIntakePath = follower
            .pathBuilder()
            .addPath(new BezierLine(p8s, p8e))
            .addPath(new BezierLine(p9s, p9e))
            .addPath(new BezierLine(p10s, p10e))
            .setTangentHeadingInterpolation()
            .build();

        Pose p11a = new Pose(120.000, 61.000);
        Pose p11b = new Pose(92.000, 61.000);
        Pose p11c = new Pose(94.000, 92.000);
        if (!red) { p11a = mirrorPoseX(p11a); p11b = mirrorPoseX(p11b); p11c = mirrorPoseX(p11c); }
        Path11 = follower
            .pathBuilder()
            .addPath(new BezierCurve(p11a, p11b, p11c))
            .setLinearHeadingInterpolation(
                red ? Math.toRadians(0)  : mirrorHeading(Math.toRadians(0)),
                red ? Math.toRadians(45) : mirrorHeading(Math.toRadians(45))
            )
            .build();

        Pose p12s = new Pose(94.000, 92.000);
        Pose p12e = new Pose(94.000, 80.000);
        if (!red) { p12s = mirrorPoseX(p12s); p12e = mirrorPoseX(p12e); }
        Path12 = follower
            .pathBuilder()
            .addPath(new BezierLine(p12s, p12e))
            .setConstantHeadingInterpolation(
                red ? Math.toRadians(45) : mirrorHeading(Math.toRadians(45))
            )
            .build();
    }
}
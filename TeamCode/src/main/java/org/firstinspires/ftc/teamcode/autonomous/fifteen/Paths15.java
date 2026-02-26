package org.firstinspires.ftc.teamcode.autonomous.fifteen;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.autonomous.AllianceTransform;
import org.firstinspires.ftc.teamcode.constant.AllianceColor;

public class Paths15 {
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13, Path14, Path15, Path16, Path17, Path18, Path19, Path20;
    public Paths15(Follower follower, AllianceColor allianceColor) {
        AllianceTransform T = new AllianceTransform(allianceColor);

        Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                    T.pose(116.800, 130.350),
                    T.pose(92.480, 84.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(38.177), T.headingDeg(0))
            .setBrakingStrength(1)
            .build();

        Path2 = follower.pathBuilder().addPath(
                new BezierCurve(
                    T.pose(92.480, 84.000),
                    T.pose(94.000, 60.000),
                    T.pose(128.000, 60.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(0))
            .build();

        Path3 = follower.pathBuilder().addPath(
                new BezierCurve(
                    T.pose(128.000, 60.000),
                    T.pose(94.000, 60.000),
                    T.pose(92.480, 84.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(0))
            .build();

        Path4 = follower.pathBuilder().addPath(
                new BezierCurve(
                    T.pose(92.480, 84.000),
                    T.pose(102.000, 64.000),
                    T.pose(131.000, 62.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(27.12))
            .build();

        Path5 = follower.pathBuilder().addPath(
                new BezierCurve(
                    T.pose(131.000, 62.000),
                    T.pose(102.000, 64.000),
                    T.pose(92.480, 84.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(27.12), T.headingDeg(0))
            .build();

        Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                    T.pose(92.480, 84.000),
                    T.pose(123.000, 84.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(0))
            .build();

        Path7 = follower.pathBuilder().addPath(
                new BezierLine(
                    T.pose(123.000, 84.000),
                    T.pose(92.480, 84.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(0))
            .build();

        Path8 = follower.pathBuilder().addPath(
                new BezierCurve(
                    T.pose(92.480, 84.000),
                    T.pose(92.003, 36.006),
                    T.pose(92.006, 36.006),
                    T.pose(128.000, 36.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(0))
            .build();

        Path9 = follower.pathBuilder().addPath(
                new BezierLine(
                    T.pose(128.000, 36.000),
                    T.pose(90.000, 86.000)
                )
            ).setTangentHeadingInterpolation()
            .setReversed()
            .setBrakingStrength(1)
            .build();
    }
}

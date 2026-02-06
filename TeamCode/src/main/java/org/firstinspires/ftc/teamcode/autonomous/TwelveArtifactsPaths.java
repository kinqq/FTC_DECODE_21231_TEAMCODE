package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.constant.AllianceColor;

public class TwelveArtifactsPaths {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;

    public TwelveArtifactsPaths(Follower follower, AllianceColor allianceColor) {
        AllianceTransform T = new AllianceTransform(allianceColor);

        Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                    T.pose(116.800, 130.350),

                    T.pose(94.480, 86.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(38.177), T.headingDeg(45))

            .build();

        Path2 = follower.pathBuilder().addPath(
                new BezierLine(
                    T.pose(94.480, 86.000),

                    T.pose(98.000, 64.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(45), T.headingDeg(0))

            .build();

        Path3 = follower.pathBuilder().addPath(
                new BezierLine(
                    T.pose(98.000, 64.000),

                    T.pose(130.000, 64.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(0))

            .build();

        Path4 = follower.pathBuilder().addPath(
                new BezierCurve(
                    T.pose(130.000, 64.000),
                    T.pose(92.000, 60.000),
                    T.pose(94.480, 86.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(0))

            .build();

        Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                    T.pose(94.480, 86.000),

                    T.pose(125.000, 84.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(0))

            .build();

        Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                    T.pose(125.000, 84.000),

                    T.pose(94.480, 86.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(0))

            .build();

        Path7 = follower.pathBuilder().addPath(
                new BezierCurve(
                    T.pose(94.480, 86.000),
                    T.pose(95.000, 35.000),
                    T.pose(90.000, 35.000),
                    T.pose(130.000, 34.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(0))

            .build();

        Path8 = follower.pathBuilder().addPath(
                new BezierLine(
                    T.pose(130.000, 34.000),

                    T.pose(94.480, 86.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(45))

            .build();

        Path9 = follower.pathBuilder().addPath(
                new BezierLine(
                    T.pose(94.480, 86.000),

                    T.pose(116.800, 130.350)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(45), T.headingDeg(38.177))

            .build();
    }
}

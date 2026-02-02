package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

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

    public TwelveArtifactsPaths(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(116.800, 130.350),
                    new Pose(92.480, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(38.177), Math.toRadians(45))
            .build();

        Path2 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(92.480, 84.000),
                    new Pose(98.000, 63.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
            .build();

        Path3 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(98.000, 63.000),
                    new Pose(130.000, 63.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

        Path4 = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(130.000, 63.000),
                    new Pose(102.000, 64.000),
                    new Pose(92.480, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

        Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(92.480, 84.000),
                    new Pose(125.000, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

        Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(125.000, 84.000),
                    new Pose(92.480, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

        Path7 = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(92.480, 84.000),
                    new Pose(98.522, 35.613),
                    new Pose(101.056, 34.621),
                    new Pose(125.000, 36.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

        Path8 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(125.000, 36.000),
                    new Pose(92.480, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
            .build();

        Path9 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(92.480, 84.000),
                    new Pose(96.000, 80.000)
                )
            ).setConstantHeadingInterpolation(Math.toRadians(45))
            .build();
    }
}

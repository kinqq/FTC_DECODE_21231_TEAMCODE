package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;


public class NineArtifactsPaths {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;

    public NineArtifactsPaths(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(116.8, 130.350),
                    new Pose(92.48, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(38.177), Math.toRadians(45))

            .build();

        Path2 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(92.48, 84.000),
                    new Pose(95.000, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
            .build();

        Path3 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(95.000, 84.000),
                    new Pose(125.000, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

        Path4 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(125.000, 84.000),
                    new Pose(92.48, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

            .build();

        Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(92.48, 84.000),
                    new Pose(95.00, 60.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

            .build();

        Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(95.00, 60.000),
                    new Pose(130.000, 60.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

        Path7 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(130.000, 60.000),
                    new Pose(92.48, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

            .build();

        Path8 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(92.48, 84.000),
                    new Pose(125.000, 60.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(27.12))
            .build();
    }
}

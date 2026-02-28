package org.firstinspires.ftc.teamcode.autonomous.fifteenfar;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.autonomous.AllianceTransform;
import org.firstinspires.ftc.teamcode.constant.AllianceColor;

public class Paths15Far {
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;
    public Paths15Far(Follower follower, AllianceColor allianceColor) {
        AllianceTransform T = new AllianceTransform(allianceColor);

        Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                    T.pose(88.000, 8.000),
                    T.pose(88.000, 20.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(0))
            .build();

        Path2 = follower.pathBuilder().addPath(
                new BezierCurve(
                    T.pose(88.000, 20.000),
                    T.pose(88.049, 36.015),
                    T.pose(130.000, 36.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(0))
            .build();

        Path3 = follower.pathBuilder().addPath(
                new BezierLine(
                    T.pose(130.000, 36.000),
                    T.pose(88.000, 20.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(0))
            .build();

        Path4 = follower.pathBuilder().addPath(
                new BezierLine(
                    T.pose(88.000, 20.000),
                    T.pose(134.000, 8.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(0))
            .build();

        Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                    T.pose(134.000, 8.000),
                    T.pose(88.000, 20.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(0))
            .build();

        Path6 = follower.pathBuilder().addPath(
                new BezierCurve(
                    T.pose(88.000, 20.000),
                    T.pose(130.000, 22.000),
                    T.pose(130.000, 22.000),
                    T.pose(132.000, 48.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(40))
            .build();

        Path7 = follower.pathBuilder().addPath(
                new BezierLine(
                    T.pose(132.000, 48.000),
                    T.pose(88.000, 20.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(40), T.headingDeg(0))
            .build();

        Path8 = follower.pathBuilder().addPath(
                new BezierCurve(
                    T.pose(88.000, 20.000),
                    T.pose(130.000, 22.000),
                    T.pose(130.000, 22.000),
                    T.pose(132.000, 48.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(0), T.headingDeg(40))
            .build();

        Path9 = follower.pathBuilder().addPath(
                new BezierLine(
                    T.pose(132.000, 48.000),
                    T.pose(88.000, 20.000)
                )
            ).setLinearHeadingInterpolation(T.headingDeg(40), T.headingDeg(45))
            .build();
    }
}

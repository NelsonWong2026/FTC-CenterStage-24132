package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String args[]) {
        SampleMecanumDrive drive = new SampleMecanumDrive();

        MeepMeep mm = new MeepMeep(800);

        Pose2d startPose = new Pose2d(12, -60, Math.toRadians(90));



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(driveShim ->
                        driveShim.trajectorySequenceBuilder(startPose)
                                /*
                                .lineToLinearHeading(new Pose2d(12, -34, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(12, -48, Math.toRadians(90)))
                                .splineToLinearHeading(new Pose2d(44, -47, Math.toRadians(0)), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(44, -35, Math.toRadians(0)))
                                .splineToSplineHeading(new Pose2d(41, -12, Math.toRadians(90)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(59, -12, Math.toRadians(90)), Math.toRadians(0))
                                 */
                                /*.splineToLinearHeading(new Pose2d(10, -31, Math.toRadians(180)), Math.toRadians(230))
                                .lineToSplineHeading(new Pose2d(15, -31, Math.toRadians(180)))
                                .splineToSplineHeading(new Pose2d(44, -40, Math.toRadians(0)), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(44, -35, Math.toRadians(0)))
                                .splineToSplineHeading(new Pose2d(41, -12, Math.toRadians(90)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(59, -12, Math.toRadians(90)), Math.toRadians(0))
                               */
                                .splineToLinearHeading(new Pose2d(14, -31, Math.toRadians(0)), Math.toRadians(90))
                                .build()
                );

        mm.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
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

        Pose2d startPose = new Pose2d(-36, 60, Math.toRadians(-90));



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.93)
                .followTrajectorySequence(driveShim ->
                        driveShim.trajectorySequenceBuilder(startPose)

                                /*.splineTo(new Vector2d(12, -34), Math.toRadians(90))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(44, -36, Math.toRadians(0)), Math.toRadians(0))
                                .addDisplacementMarker(() -> {

                                })
                                .splineToLinearHeading(new Pose2d(41, -60, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(59, -60, Math.toRadians(0)), Math.toRadians(0))*/

                               /* .splineToSplineHeading(new Pose2d(10, -31, Math.toRadians(180)), Math.toRadians(180))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(44, -35, Math.toRadians(0)), Math.toRadians(0))
                                .addDisplacementMarker(() -> {

                                })
                                .splineToLinearHeading(new Pose2d(41, -60, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(59, -60, Math.toRadians(0)), Math.toRadians(0))*/

                                /*.splineTo(new Vector2d(12, -34.5), Math.toRadians(90))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(44, -35, Math.toRadians(0)), Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                   // arm.setArmPos(2500);
                                    //claw.setPivotPower(1);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(4, () -> {
                                   // claw.setPivotPower(0);
                                    //arm.setArmPos(-2000);
                                })
                                .waitSeconds(6)
                                .splineToLinearHeading(new Pose2d(41, -60, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(59, -60, Math.toRadians(0)), Math.toRadians(0))*/
                                .splineTo(new Vector2d(-36, 35), Math.toRadians(-90))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-36, 60, Math.toRadians(0)), Math.toRadians(0))
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(16, 60), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(19, 35, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(48, 35, Math.toRadians(0)), Math.toRadians(0))
                                .build()
                );

        mm.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
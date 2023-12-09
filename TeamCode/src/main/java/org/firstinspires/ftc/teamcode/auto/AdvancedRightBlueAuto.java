package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RoadRunnerUtil.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunnerUtil.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.StartingConfiguration;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.vision.ContourDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous(name = "Advanced Right Blue Auto", group = "advanced auto")
public class AdvancedRightBlueAuto extends OpMode {
    private VisionPortal visionPortal;
    private ContourDetectionProcessor contourDetectionProcessor;
    public static int lowerBlueHue = 105, upperBlueHue = 130, lowerBlueV = 110, upperBlueV = 255;
    private SampleMecanumDrive drive;
    private Arm arm = new Arm();
    private Claw claw = new Claw();


    @Override
    public void init() {
        Scalar lower = new Scalar(lowerBlueHue, 100, lowerBlueV);
        Scalar upper = new Scalar(upperBlueHue, 255, upperBlueV);
        double minArea = 6500;
        contourDetectionProcessor = new ContourDetectionProcessor(
                lower,
                upper,
                () -> minArea,
                () -> 410,
                () -> 410
        );

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Constants.Vision.camera1))
                .addProcessor(contourDetectionProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        FtcDashboard.getInstance().startCameraStream(contourDetectionProcessor, 30);

        arm.init(hardwareMap);
        claw.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        arm.zeroCalibrate();
    }

    @Override
    public void init_loop() {

        telemetry.addData("Currently Recorded Position", contourDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + contourDetectionProcessor.getLargestContourX() + ", y: " + contourDetectionProcessor.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", contourDetectionProcessor.getLargestContourArea());
        telemetry.update();
    }

    @Override
    public void start() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }
        ContourDetectionProcessor.PropPositions recordedPropPosition = contourDetectionProcessor.getRecordedPropPosition();

        if (recordedPropPosition == ContourDetectionProcessor.PropPositions.UNFOUND) {
            recordedPropPosition = ContourDetectionProcessor.PropPositions.LEFT;
        }

        Pose2d rightStartPose = new Pose2d(-36, 60, Math.toRadians(-90));

        drive.setPoseEstimate(rightStartPose);
        switch (recordedPropPosition) {
            case LEFT:
                TrajectorySequence rightLeftTraj = drive.trajectorySequenceBuilder(rightStartPose)
                        .turn(Math.toRadians(35))
                        .lineToLinearHeading(new Pose2d(-34, 26, Math.toRadians(0)))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-36, 60, Math.toRadians(0)), Math.toRadians(0))
                        .setReversed(false)
                        .waitSeconds(15)
                        .splineToConstantHeading(new Vector2d(16, 60), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(19, 35, Math.toRadians(0)), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(48.5, 35, Math.toRadians(0)), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            arm.setArmPos(2500);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            claw.setPivotPower(0);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                            claw.setPivotPower(1);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                            claw.setPivotPower(0);
                            arm.setArmPos(-2000);
                        })
                        .build();
                drive.followTrajectorySequence(rightLeftTraj);

                break;
            case MIDDLE:
                TrajectorySequence rightMiddleTraj = drive.trajectorySequenceBuilder(rightStartPose)
                        .splineTo(new Vector2d(-36, 35), Math.toRadians(-90))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-36, 60, Math.toRadians(0)), Math.toRadians(0))
                        .setReversed(false)
                        .waitSeconds(15)
                        .splineToConstantHeading(new Vector2d(16, 60), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(19, 35, Math.toRadians(0)), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(48.5, 35, Math.toRadians(0)), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            arm.setArmPos(2500);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            claw.setPivotPower(0);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                            claw.setPivotPower(1);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                            claw.setPivotPower(0);
                            arm.setArmPos(-2000);
                        })
                        .build();
                drive.followTrajectorySequence(rightMiddleTraj);
                break;
            case RIGHT:
                TrajectorySequence rightRightTraj = drive.trajectorySequenceBuilder(rightStartPose)
                        .splineTo(new Vector2d(-36, 34.5), Math.toRadians(-90))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-36, 60, Math.toRadians(0)), Math.toRadians(0))
                        .setReversed(false)
                        .waitSeconds(15)
                        .splineToConstantHeading(new Vector2d(16, 60), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(19, 35, Math.toRadians(0)), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(48.5, 35, Math.toRadians(0)), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            arm.setArmPos(2500);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            claw.setPivotPower(0);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                            claw.setPivotPower(1);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                            claw.setPivotPower(0);
                            arm.setArmPos(-2000);
                        })
                        .build();
                drive.followTrajectorySequence(rightRightTraj);
                break;
        }



    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        contourDetectionProcessor.close();
        visionPortal.close();
    }
}

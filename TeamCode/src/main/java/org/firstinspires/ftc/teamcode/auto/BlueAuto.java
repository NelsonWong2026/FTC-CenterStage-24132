package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RoadRunnerUtil.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunnerUtil.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.StartingConfiguration;
import org.firstinspires.ftc.teamcode.vision.ContourDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;


@Config
@Autonomous(name = "Blue Right Main Auto", group = "auto")
public class BlueAuto extends OpMode {
    private VisionPortal visionPortal;
    private ContourDetectionProcessor contourDetectionProcessor;
    public static int lowerBlueHue = 100, upperBlueHue = 140;
    private SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    private StartingConfiguration.AlliancePosition setAlliancePos;
    private StartingConfiguration configStartingPos = new StartingConfiguration();


    @Override
    public void init() {
        Scalar lower = new Scalar(lowerBlueHue, 100, 100);
        Scalar upper = new Scalar(upperBlueHue, 255, 255);
        double minArea = 100;
        contourDetectionProcessor = new ContourDetectionProcessor(
                lower,
                upper,
                () -> minArea,
                () -> 213,
                () -> 426
        );

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Constants.Vision.camera1))
                .addProcessor(contourDetectionProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        FtcDashboard.getInstance().startCameraStream(contourDetectionProcessor, 30);
    }

    @Override
    public void init_loop() {
        configStartingPos.startConfiguration(gamepad1, setAlliancePos);

        telemetry.addData("Currently Recorded Position", contourDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + contourDetectionProcessor.getLargestContourX() + ", y: " + contourDetectionProcessor.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", contourDetectionProcessor.getLargestContourArea());
        telemetry.addData("Alliance Position: ", setAlliancePos);
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
            recordedPropPosition = ContourDetectionProcessor.PropPositions.MIDDLE;
        }

        switch (setAlliancePos) {
            case LEFT:
                Pose2d startPose = new Pose2d(12, 60, Math.toRadians(-90));

                drive.setPoseEstimate(startPose);
                switch (recordedPropPosition) {
                    case LEFT:
                        TrajectorySequence leftLeftTrajSeq = drive.trajectorySequenceBuilder(startPose)
                                .splineToSplineHeading(new Pose2d(10, 31, Math.toRadians(180)), Math.toRadians(180))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(44, 35, Math.toRadians(0)), Math.toRadians(0))
                                .addDisplacementMarker(() -> {

                                })
                                .splineToLinearHeading(new Pose2d(41, 60, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(59, 60, Math.toRadians(0)), Math.toRadians(0))
                                .build();
                        drive.followTrajectorySequence(leftLeftTrajSeq);
                        break;
                    case MIDDLE:
                        TrajectorySequence leftMiddleTrajSeq = drive.trajectorySequenceBuilder(startPose)
                                .splineTo(new Vector2d(12, 34), Math.toRadians(-90))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(44, 36, Math.toRadians(0)), Math.toRadians(0))
                                .addDisplacementMarker(() -> {

                                })
                                .splineToLinearHeading(new Pose2d(41, 60, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(59, 60, Math.toRadians(0)), Math.toRadians(0))
                                .build();

                        drive.followTrajectorySequence(leftMiddleTrajSeq);

                        break;
                    case RIGHT:
                        TrajectorySequence leftRightTrajSeq = drive.trajectorySequenceBuilder(startPose)
                                .lineToConstantHeading(new Vector2d(22, 39))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(33, 55), Math.toRadians(-45))
                                .splineToSplineHeading(new Pose2d(44, 35, Math.toRadians(0)), Math.toRadians(0))
                                .addDisplacementMarker(() -> {

                                })
                                .splineToLinearHeading(new Pose2d(41, 60, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(59, 60, Math.toRadians(0)), Math.toRadians(0))
                                .build();
                        drive.followTrajectorySequence(leftRightTrajSeq);
                        break;
                }
            case RIGHT:
                switch (recordedPropPosition) {
                    case LEFT:
                        break;

                    case MIDDLE:
                        break;

                    case RIGHT:
                        break;

                }
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

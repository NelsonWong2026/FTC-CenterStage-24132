package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RoadRunnerUtil.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunnerUtil.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.StartingConfiguration;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.PIDF_Arm;
import org.firstinspires.ftc.teamcode.vision.ContourDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Config
@Autonomous(name = "Red Left Main Auto", group = "auto")
public class RedAuto extends OpMode {
    private VisionPortal visionPortal;
    private ContourDetectionProcessor contourDetectionProcessor;
    public static int lowerRedHue = 153, upperRedHue = 180, lowerBlueHue = 100, upperBlueHue = 140;;
    private SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    private PIDF_Arm arm = new PIDF_Arm();
    private Claw claw = new Claw();
    private StartingConfiguration.AlliancePosition setAlliancePos;
    private StartingConfiguration configStartingPos = new StartingConfiguration();

    @Override
    public void init() {
        Scalar lower = new Scalar(lowerRedHue, 100, 100);
        Scalar upper = new Scalar(upperRedHue, 255, 255);
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

        arm.init(hardwareMap);
        claw.init(hardwareMap);
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
            case RIGHT:
                Pose2d startPose = new Pose2d(12, 60, Math.toRadians(-90));

                drive.setPoseEstimate(startPose);
                switch (recordedPropPosition) {
                    case LEFT:
                        TrajectorySequence rightLeftTrajSeq = drive.trajectorySequenceBuilder(startPose)
                                .splineToSplineHeading(new Pose2d(10, 31, Math.toRadians(180)), Math.toRadians(180))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(44, 35, Math.toRadians(0)), Math.toRadians(0))
                                .addDisplacementMarker(() -> {

                                })
                                .splineToLinearHeading(new Pose2d(41, 60, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(59, 60, Math.toRadians(0)), Math.toRadians(0))
                                .build();
                        drive.followTrajectorySequence(rightLeftTrajSeq);
                        break;
                    case MIDDLE:
                        TrajectorySequence rightMiddleTrajSeq = drive.trajectorySequenceBuilder(startPose)
                                .splineTo(new Vector2d(12, 34), Math.toRadians(90))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(44, 36, Math.toRadians(0)), Math.toRadians(0))
                                .addDisplacementMarker(() -> {

                                })
                                .splineToLinearHeading(new Pose2d(41, 60, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(59, 60, Math.toRadians(0)), Math.toRadians(0))
                                .build();

                        drive.followTrajectorySequence(rightMiddleTrajSeq);

                        break;
                    case RIGHT:
                        TrajectorySequence rightRightTrajSeq = drive.trajectorySequenceBuilder(startPose)
                        .lineToConstantHeading(new Vector2d(22, 39))
                            .setReversed(true)
                            .splineToConstantHeading(new Vector2d(33, 55), Math.toRadians(45))
                            .splineToSplineHeading(new Pose2d(44, 35, Math.toRadians(0)), Math.toRadians(0))
                            .addDisplacementMarker(() -> {

                            })
                            .splineToLinearHeading(new Pose2d(41, 60, Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(59, 60, Math.toRadians(0)), Math.toRadians(0))
                            .build();

                        drive.followTrajectorySequence(rightRightTrajSeq);
                        break;
                }
            case LEFT:
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

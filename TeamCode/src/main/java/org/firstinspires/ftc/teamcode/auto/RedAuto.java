package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
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
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.PIDF_Arm;
import org.firstinspires.ftc.teamcode.vision.ContourDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Config
@Autonomous(name = "Red Main Auto", group = "main auto")
public class RedAuto extends OpMode {
    private VisionPortal visionPortal;
    private ContourDetectionProcessor contourDetectionProcessor;
    public static int lowerRedHue = 153, upperRedHue = 180, lowerRedV = 65, upperRedV = 255;
    private SampleMecanumDrive drive;
    private Arm arm = new Arm();
    private Claw claw = new Claw();
    private StartingConfiguration.AlliancePosition setAlliancePos;
    private StartingConfiguration configStartingPos = new StartingConfiguration();
    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;

    public RedAuto() {
    }

    @Override
    public void init() {
        Scalar lower = new Scalar(lowerRedHue, 100, lowerRedV);
        Scalar upper = new Scalar(upperRedHue, 255, upperRedV);
        double minArea = 5000;
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
        setAlliancePos = configStartingPos.startConfiguration(gamepad1, gamepad2);

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
            recordedPropPosition = ContourDetectionProcessor.PropPositions.LEFT;
        }

        switch (setAlliancePos) {
            case RIGHT:
                Pose2d rightStartPose = new Pose2d(12, -60, Math.toRadians(90));

                drive.setPoseEstimate(rightStartPose);
                switch (recordedPropPosition) {
                    case LEFT:
                        TrajectorySequence rightLeftTrajSeq = drive.trajectorySequenceBuilder(rightStartPose)
                                .splineToLinearHeading(new Pose2d(10, -31, Math.toRadians(180)), Math.toRadians(180))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(48.5, -30, Math.toRadians(0)), Math.toRadians(0))
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
                                .waitSeconds(6)
                                .splineToLinearHeading(new Pose2d(41, -60, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(59, -60.5, Math.toRadians(0)), Math.toRadians(0))
                                .build();
                        drive.followTrajectorySequence(rightLeftTrajSeq);
                        break;
                    case MIDDLE:
                        TrajectorySequence rightMiddleTrajSeq = drive.trajectorySequenceBuilder(rightStartPose)
                                .splineTo(new Vector2d(12, -34.5), Math.toRadians(90))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(48.5, -35, Math.toRadians(0)), Math.toRadians(0))
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
                                .waitSeconds(6)
                                .splineToLinearHeading(new Pose2d(41, -60, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(59, -60.5, Math.toRadians(0)), Math.toRadians(0))
                                .build();

                        drive.followTrajectorySequence(rightMiddleTrajSeq);

                        break;
                    case RIGHT:
                        TrajectorySequence rightRightTrajSeq = drive.trajectorySequenceBuilder(rightStartPose)
                                .turn(Math.toRadians(-35))
                                .lineToLinearHeading(new Pose2d(13.5, -30, Math.toRadians(0)))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(33, -55), Math.toRadians(45))
                                .splineToSplineHeading(new Pose2d(48.5, -39, Math.toRadians(0)), Math.toRadians(0))
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
                                .waitSeconds(6)
                                .splineToLinearHeading(new Pose2d(41, -60, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(59, -60.5, Math.toRadians(0)), Math.toRadians(0))
                                .build();

                        drive.followTrajectorySequence(rightRightTrajSeq);
                        break;
                }
                break;
            case LEFT:
                Pose2d leftStartPose = new Pose2d(-36, -60, Math.toRadians(90));

                drive.setPoseEstimate(leftStartPose);
                switch (recordedPropPosition) {
                    case LEFT:
                        TrajectorySequence leftLeftTrajSeq = drive.trajectorySequenceBuilder(leftStartPose)
                                .splineToLinearHeading(new Pose2d(-37.5, -31, Math.toRadians(180)), Math.toRadians(180))
                                .back(5)
                                .build();
                        drive.followTrajectorySequence(leftLeftTrajSeq);
                        break;

                    case MIDDLE:
                        TrajectorySequence leftMiddleTrajSeq = drive.trajectorySequenceBuilder(leftStartPose)
                                .splineTo(new Vector2d(-36, -34.5), Math.toRadians(90))
                                .back(5)
                                .build();
                        drive.followTrajectorySequence(leftMiddleTrajSeq);
                        break;

                    case RIGHT:
                        TrajectorySequence leftRightTrajSeq = drive.trajectorySequenceBuilder(leftStartPose)
                                .turn(Math.toRadians(-35))
                                .lineToLinearHeading(new Pose2d(-34.5, -30, Math.toRadians(0)))
                                .back(5)
                                .build();
                        drive.followTrajectorySequence(leftRightTrajSeq);
                        break;

                }
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

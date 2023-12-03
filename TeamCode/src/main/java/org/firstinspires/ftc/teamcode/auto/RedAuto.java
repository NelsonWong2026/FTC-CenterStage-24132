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
    private StartingConfiguration.AllianceColor setAllianceColor;
    private StartingConfiguration.AlliancePosition setAlliancePos;
    private RevTouchSensor touchSensor;
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
        configStartingPos.startConfiguration(gamepad1, setAllianceColor, setAlliancePos);
        if (setAllianceColor == StartingConfiguration.AllianceColor.BLUE_ALLIANCE) {

        }
        telemetry.addData("Currently Recorded Position", contourDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + contourDetectionProcessor.getLargestContourX() + ", y: " + contourDetectionProcessor.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", contourDetectionProcessor.getLargestContourArea());
        telemetry.addData("Alliance Color: ", setAllianceColor);
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

        switch (setAllianceColor) {
            case RED_ALLIANCE:
                Pose2d startPose = new Pose2d(12, -60, Math.toRadians(90));

                drive.setPoseEstimate(startPose);
                switch (recordedPropPosition) {
                    case LEFT:
                        Trajectory leftTraj1 = drive.trajectoryBuilder(startPose)

                                .build();
                        break;
                    case MIDDLE:
                        Trajectory middleRedTraj1 = drive.trajectoryBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(12, -34, Math.toRadians(90)))
                                .build();
                        Trajectory middleRedTraj2 = drive.trajectoryBuilder(middleRedTraj1.end())
                                .lineToSplineHeading(new Pose2d(12, -48, Math.toRadians(90)))
                                .splineToLinearHeading(new Pose2d(44, -47, Math.toRadians(0)), Math.toRadians(0))
                                .build();
                        Trajectory middleRedTraj3 = drive.trajectoryBuilder(middleRedTraj2.end())
                                .lineToLinearHeading(new Pose2d(44, -35, Math.toRadians(0)))
                                .build();
                        Trajectory middleRedTraj4 = drive.trajectoryBuilder(middleRedTraj3.end())
                                .splineToSplineHeading(new Pose2d(41, -12, Math.toRadians(90)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(59, -12, Math.toRadians(90)), Math.toRadians(0))
                                .build();

                        drive.followTrajectory(middleRedTraj1);
                        drive.followTrajectory(middleRedTraj2);
                        drive.followTrajectory(middleRedTraj3);
                        arm.setTarget(0);
                        claw.setPivotPower(-1);
                        claw.setPivotPower(1);
                        arm.setTarget(0);
                        drive.followTrajectory(middleRedTraj4);
                        break;
                    case RIGHT:

                        break;
                }
            case BLUE_ALLIANCE:
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

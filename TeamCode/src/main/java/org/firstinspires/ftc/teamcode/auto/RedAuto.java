package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.vision.ContourDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Config
@Autonomous(name = "Red Left Main Auto", group = "auto")
public class RedAuto extends OpMode {
    private VisionPortal visionPortal;
    private ContourDetectionProcessor contourDetectionProcessor;
    public static int lowerRedHue = 153, upperRedHue = 180;
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
            recordedPropPosition = ContourDetectionProcessor.PropPositions.MIDDLE;
        }

        switch (recordedPropPosition) {
            case LEFT:

                break;
            case UNFOUND:
            case MIDDLE:

                break;
            case RIGHT:

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

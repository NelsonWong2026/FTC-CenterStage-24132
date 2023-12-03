package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;

public class CalibrateArm extends LinearOpMode {
    private RevTouchSensor touchSensor;
    private DcMotor arm;
    @Override
    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.get(DcMotor.class, Constants.Arm.Arm);
        touchSensor = hardwareMap.get(RevTouchSensor.class, Constants.Arm.touchSensor);

        waitForStart();

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(Constants.Arm.CalibratePower);
        while (opModeIsActive()) {
            if (touchSensor.isPressed()) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        arm.setPower(0);
    }
}

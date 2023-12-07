package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp(name = "CalibrateArm")
public class CalibrateArm extends LinearOpMode {
    private RevTouchSensor touchSensor;
    private DcMotor arm;
    @Override
    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.get(DcMotor.class, Constants.Arm.Arm);

        waitForStart();

        while (opModeIsActive()) {
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (gamepad1.a) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            }
            telemetry.addData("Arm Position: ", arm.getCurrentPosition());
        }
    }
}

package org.firstinspires.ftc.teamcode.subsystem;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;

@Config
//@Disabled
@TeleOp(name = "ArmTest", group = "OpMode")
public class ArmTest extends OpMode {
    public static int target = 0;
    private DcMotor arm_motor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotor.class, Constants.Arm.Arm);
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        int armPos = arm_motor.getCurrentPosition();
        arm_motor.setTargetPosition(target);
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_motor.setPower(0.4);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}


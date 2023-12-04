package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Arm {
    private DcMotorEx arm;
    private RevTouchSensor touchSensor;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    public Arm() {

    }

    public void init(HardwareMap hwMap) {
        this.arm = hwMap.get(DcMotorEx.class, Constants.Arm.Arm);
        this.arm.setDirection(DcMotor.Direction.FORWARD);
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.touchSensor = hwMap.get(RevTouchSensor.class, Constants.Arm.);
    }

    public void zeroCalibrate() {
        while (!touchSensor.isPressed()) {
            arm.setPower(-0.3);
        }
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setControl(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            arm.setTargetPosition();
        }
        else if (gamepad.dpad_down) {
            arm.setTargetPosition();
        }
        else if (gamepad.dpad_right) {
            arm.setTargetPosition();
        }
    }
}

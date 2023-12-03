package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Arm {
    private DcMotor arm;
    private RevTouchSensor touchSensor;

    public Arm() {

    }

    public void init(HardwareMap hwMap) {
        this.arm = hwMap.get(DcMotor.class, Constants.Arm.Arm);
        this.arm.setDirection(DcMotor.Direction.FORWARD);
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void zeroCalibrate() {

    }

    public void setControl(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            arm.setTargetPosition();
        }
        else if (gamepad.dpad_down) {
            arm.setTargetPosition();
        }
    }
}

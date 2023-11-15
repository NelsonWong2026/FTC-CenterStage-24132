package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Arm {
    private DcMotor Arm;

    public Arm() {

    }

    public void init(HardwareMap hwMap) {
        this.Arm = hwMap.get(DcMotor.class, Constants.Arm.Arm);
        this.Arm.setDirection(DcMotor.Direction.REVERSE);
        this.Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setControl(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            Arm.setPower(1);
        }
        else if (gamepad.dpad_down) {
            Arm.setPower(-1);
        }
        else {
            Arm.setPower(0);
        }
    }
}

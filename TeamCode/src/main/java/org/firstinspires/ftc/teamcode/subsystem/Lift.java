package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Lift {
    private DcMotor Lift;

    public Lift() {

    }

    public void init(HardwareMap hwMap) {
        this.Lift = hwMap.get(DcMotor.class, Constants.Lift.Lift);
        this.Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setControl(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            Lift.setPower(1);
        }
        else if (gamepad.dpad_down) {
            Lift.setPower(-1);
        }
        else {
            Lift.setPower(0);
        }
    }
}

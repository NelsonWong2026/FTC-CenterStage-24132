package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Lift {
    private DcMotor Lift, Swing;

    public Lift() {

    }

    public void init(HardwareMap hwMap) {
        this.Lift = hwMap.get(DcMotor.class, Constants.Lift.Lift);
        this.Swing = hwMap.get(DcMotor.class, Constants.Lift.Swing);
        this.Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.Swing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setControl(Gamepad gamepad) {
        Swing.setPower(gamepad.right_stick_y);
        if (gamepad.dpad_up) {
            Lift.setPower(1);
        }
        else if (gamepad.dpad_down) {
            Lift.setPower(-1);
        }
        /*else if (gamepad.x) {
            Swing.setPower(1);
        }
        else if (gamepad.y) {
            Swing.setPower(-1);
        }*/
        else {
            Lift.setPower(0);
            Swing.setPower(0);
        }
    }
}

package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Lift {
    private DcMotor leftLift, rightLift;

    public Lift() {

    }

    public void init(HardwareMap hwMap) {
        this.leftLift = hwMap.get(DcMotor.class, Constants.Lift.leftLift);
        this.rightLift = hwMap.get(DcMotor.class, Constants.Lift.rightLift);
        this.leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setControl(Gamepad gamepad) {
        if (gamepad.a) {
            leftLift.setPower(1);
            rightLift.setPower(1);
        }
        else if (gamepad.b) {
            leftLift.setPower(-1);
            rightLift.setPower(-1);
        }
        else {
            leftLift.setPower(0);
            rightLift.setPower(0);
        }
    }

}

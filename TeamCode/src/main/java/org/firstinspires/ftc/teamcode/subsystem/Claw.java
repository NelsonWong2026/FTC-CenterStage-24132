package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Claw {
    //private Servo Claw;
    //private Servo servoPivot;
    private CRServo pivot;

    public Claw() {

    }

    public void init(HardwareMap hwMap) {
        //this.Claw = hwMap.get(Servo.class, Constants.Claw.Claw);
        this.pivot = hwMap.get(CRServo.class, Constants.Claw.Pivot);
        //this.servoPivot = hwMap.get(Servo.class, Constants.Claw.Pivot);
    }

    /*public void setClawPos(double Position) {
        Claw.setPosition(Position);
    }*/
    public void setPivotPower(double Power) {
        pivot.setPower(Power);
    }

    public void setControl (Gamepad gamepad) {
        /*if (gamepad.dpad_up) {
            pivot.setPosition(1);
        }*/
        if (gamepad.x) {
            pivot.setPower(1);
        }
        else if (gamepad.y) {
            pivot.setPower(-1);
        }
        else {
            pivot.setPower(0);
        }
    }
}

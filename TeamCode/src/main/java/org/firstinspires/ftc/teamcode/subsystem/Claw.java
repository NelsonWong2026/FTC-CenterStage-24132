package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Claw {
    private Servo Claw;
    //private Servo servoPivot;
    private CRServo Pivot;

    public Claw() {

    }

    public void init(HardwareMap hwMap) {
        this.Claw = hwMap.get(Servo.class, Constants.Claw.Claw);
        this.Pivot = hwMap.get(CRServo.class, Constants.Claw.Pivot);
        //this.servoPivot = hwMap.get(Servo.class, Constants.Claw.Pivot);
    }

    public void setClawPos(double Position) {
        Claw.setPosition(Position);
    }
    /*public void setPivotPos(double Position) {
        Pivot.setPower(1);
    }*/

    public void setControl (Gamepad gamepad) {
        if (gamepad.a) {
            Claw.setPosition(0.5);
        }
        else if (gamepad.b) {
            Claw.setPosition(0);
        }
        else if (gamepad.x) {
            Pivot.setPower(1);
        }
        else if (gamepad.y) {
            Pivot.setPower(-1);
        }
        /*else if (gamepad.left_bumper) {
            servoPivot.setPosition(0);
        }
        else if (gamepad.right_bumper) {
            servoPivot.setPosition(1);
        }*/
        else {
            Pivot.setPower(0);

        }
    }
}

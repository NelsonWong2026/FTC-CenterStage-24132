package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Claw extends SubsystemBase {
    //private Servo Claw;
    //private Servo servoPivot;
    private CRServo Pivot;

    public Claw(final HardwareMap hwMap) {
        Pivot = hwMap.get(CRServo.class, Constants.Claw.Pivot);
    }

    public void moveUp() {
        Pivot.setPower(1);
    }

    public void setPivotPos(double Power) {
        Pivot.setPower(Power);
    }

    public void setControl (Gamepad gamepad) {
        /*if (gamepad.a) {
            Claw.setPosition(0.5);
        }
        else if (gamepad.b) {
            Claw.setPosition(0);
        }*/
        if (gamepad.x) {
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
        }
        else {
            Pivot.setPosition(0);

        }*/
    }
}

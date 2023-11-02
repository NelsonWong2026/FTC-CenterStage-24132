package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Claw {
    private Servo Claw;

    public Claw() {

    }

    public void setClawPos(double Position) {
        Claw.setPosition(Position);
    }

    public void init(HardwareMap hwMap) {
        this.Claw = hwMap.get(Servo.class, Constants.Claw.Claw);
    }

    public void setControl (Gamepad gamepad) {
        if (gamepad.a) {
            Claw.setPosition(1);
        }
        else if (gamepad.b) {
            Claw.setPosition(0);
        }
    }
}

package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Claw {
    private Servo claw;

    public Claw() {

    }

    public void init(HardwareMap hwMap) {
        this.claw = hwMap.get(Servo.class, Constants.Claw.Claw);
    }

    public void setControl(Gamepad gamepad) {
        if (gamepad.a) {
            claw.setPosition(1);
        }
        else if (gamepad.b) {
            claw.setPosition(0);
        }
    }
}

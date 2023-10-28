package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Launcher {
    private Servo Launcher;

    public Launcher() {

    }

    public void init(HardwareMap hwMap) {
        this.Launcher = hwMap.get(Servo.class, Constants.Launcher.Launcher);
    }

    public void setControl(Gamepad gamepad) {
        if (gamepad.a && gamepad.right_bumper) {
            Launcher.setPosition(1);
        }
        else if (gamepad.a && gamepad.left_bumper) {
            Launcher.setPosition(0);
        }
    }
}

package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Launcher {
    private Servo launcher;

    public Launcher() {

    }

    public void init(HardwareMap hwMap) {
        this.launcher = hwMap.get(Servo.class, Constants.Launcher.Launcher);
    }

    public void setLauncherPos(double Position) {
        launcher.setPosition(Position);
    }

    public void setControl(Gamepad gamepad) {
        if (gamepad.right_bumper) {
            launcher.setPosition(0.8);
        }
        else if (gamepad.left_bumper) {
            launcher.setPosition(0);
        }
    }
}

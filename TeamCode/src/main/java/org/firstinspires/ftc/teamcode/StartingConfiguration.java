package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class StartingConfiguration {
    private AlliancePosition position;
    public enum AlliancePosition {
        LEFT,
        RIGHT,
        PRESSTHEBUTTON;

    }

    public StartingConfiguration() {

    }

    public AlliancePosition startConfiguration(Gamepad gamepad, Gamepad gamepad2) {
        if (gamepad.dpad_left || gamepad2.dpad_left) {
            position = AlliancePosition.LEFT;
        }
        else if (gamepad.dpad_right || gamepad2.dpad_right) {
            position = AlliancePosition.RIGHT;
        }

        return position;
    }
}

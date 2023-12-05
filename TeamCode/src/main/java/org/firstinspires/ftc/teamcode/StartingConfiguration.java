package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class StartingConfiguration {
    public enum AlliancePosition {
        LEFT,
        RIGHT;

    }

    public StartingConfiguration() {

    }

    public void startConfiguration(Gamepad gamepad, AlliancePosition setAlliancePos) {
        if (gamepad.dpad_left) {
            setAlliancePos = AlliancePosition.LEFT;
        }
        else if (gamepad.dpad_right) {
            setAlliancePos = AlliancePosition.RIGHT;
        }
        else {
            setAlliancePos = AlliancePosition.LEFT;
        }
    }
}

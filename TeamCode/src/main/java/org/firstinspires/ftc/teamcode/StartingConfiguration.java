package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class StartingConfiguration {
    public enum AllianceColor {
        BLUE_ALLIANCE,
        RED_ALLIANCE;
    }
    public enum AlliancePosition {
        LEFT,
        RIGHT;

    }

    public StartingConfiguration() {

    }

    public void startConfiguration(Gamepad gamepad, AllianceColor setAllianceColor, AlliancePosition setAlliancePos) {
        if (gamepad.b) {
            setAllianceColor = AllianceColor.BLUE_ALLIANCE;
        }
        else if (gamepad.a) {
            setAllianceColor = AllianceColor.RED_ALLIANCE;
        }
        else if (gamepad.dpad_left) {
            setAlliancePos = AlliancePosition.LEFT;
        }
        else if (gamepad.dpad_right) {
            setAlliancePos = AlliancePosition.RIGHT;
        }
        else {
            setAllianceColor = AllianceColor.RED_ALLIANCE;
            setAlliancePos = AlliancePosition.LEFT;
        }
    }
}

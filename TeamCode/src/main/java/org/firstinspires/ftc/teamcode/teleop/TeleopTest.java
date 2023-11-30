package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Commands.MoveUp;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

public class TeleopTest extends CommandOpMode {
    private Motor fL, bL, fR, bR;
    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;
    private Claw claw;
    private GamepadEx driverOp = new GamepadEx(gamepad1);
    private GamepadEx toolOp = new GamepadEx(gamepad2);
    private Button upButton = new GamepadButton(
            driverOp, GamepadKeys.Button.X
    );
    @Override
    public void initialize() {
        claw = new Claw(hardwareMap);

        toolOp.getGamepadButton(GamepadKeys.Button.X);
        upButton.whenPressed(new MoveUp(claw));
    }
}

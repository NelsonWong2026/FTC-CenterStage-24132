package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Commands.MoveUp;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class TeleopTest extends CommandOpMode {
    private Motor fL, bL, fR, bR;
    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;
    private Claw claw;
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;
    private IMU imu;
    private IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP));
    private Button upButton = new GamepadButton(
            gamepadEx, GamepadKeys.Button.X
    );
    @Override
    public void initialize() {
        claw = new Claw(hardwareMap);
        fL = new Motor(hardwareMap, Constants.MecanumDrive.leftFront, Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, Constants.MecanumDrive.leftBack, Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, Constants.MecanumDrive.rightFront, Motor.GoBILDA.RPM_312);
        bR = new Motor(hardwareMap, Constants.MecanumDrive.rightBack, Motor.GoBILDA.RPM_312);

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        imu.initialize(parameters);
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        driveSubsystem = new DriveSubsystem(fL, bL, fR, bR);
        driveCommand = new DriveCommand(driveSubsystem, gamepadEx::getLeftX, gamepadEx::getLeftY, gamepadEx::getRightX, botHeading);

        register(driveSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);


        gamepadEx2.getGamepadButton(GamepadKeys.Button.X);
        upButton.whenPressed(new MoveUp(claw));
    }
}

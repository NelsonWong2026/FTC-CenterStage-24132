package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Constants;

public class DriveSubsystem extends SubsystemBase {
    private MecanumDrive drive;
    private Motor fL, bL, fR, bR;

    public DriveSubsystem(Motor frontLeft, Motor backLeft, Motor frontRight, Motor backRight) {
        fL = frontLeft;
        bL = backLeft;
        fR = frontRight;
        bR = backRight;
        drive = new MecanumDrive(fL, bL, fR, bR);
    }
    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed, double gyroAngle) {
        drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, gyroAngle);

    }
}

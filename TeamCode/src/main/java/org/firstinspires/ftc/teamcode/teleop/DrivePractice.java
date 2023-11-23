package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Launcher;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.PIDF_Arm;

@TeleOp(name="DrivePractice", group="DrivePractice")
public class DrivePractice extends OpMode {
    private MecanumDrive drive = new MecanumDrive();
    private boolean fieldCentric = true;

    @Override
    public void init() {
        this.drive.init(hardwareMap);
        telemetry.addData("Status","Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        this.drive.setControl(gamepad1, fieldCentric);

        telemetry.addData("Status","Enabled");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("Status","Stopped");
        telemetry.update();
    }
}
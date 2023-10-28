package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Launcher;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;

@TeleOp(name="Main Teleop Mode (No Timer)", group="OpMode")
public class MainTeleopNoTimer extends OpMode {
    private MecanumDrive drive = new MecanumDrive();
    private Claw claw = new Claw();
    private Lift lift = new Lift();
    private Launcher launcher = new Launcher();

    @Override
    public void init() {
        this.drive.init(hardwareMap);
        this.claw.init(hardwareMap);
        this.lift.init(hardwareMap);
        this.launcher.init(hardwareMap);
        telemetry.addData("Status","Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        this.drive.setControl(gamepad1);
        this.claw.setControl(gamepad2);
        this.lift.setControl(gamepad2);
        this.launcher.setControl(gamepad2);

        telemetry.addData("Status","Enabled");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("Status","Stopped");
        telemetry.update();
    }
}
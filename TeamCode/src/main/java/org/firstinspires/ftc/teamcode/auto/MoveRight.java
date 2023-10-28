package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;

@Autonomous(name = "MoveRight", group = "auto")
public class MoveRight extends LinearOpMode {
    MecanumDrive drive = new MecanumDrive();
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);

        waitForStart();
        drive.strafeRight(0.7);

        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < 5000)) {

        }

        drive.stop();
    }
}

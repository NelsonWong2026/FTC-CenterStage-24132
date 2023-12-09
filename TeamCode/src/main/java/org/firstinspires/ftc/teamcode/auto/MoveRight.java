package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;

@Autonomous(name = "MoveRight", group = "bad auto")
public class MoveRight extends LinearOpMode {
    MecanumDrive drive = new MecanumDrive();
    ElapsedTime runtime = new ElapsedTime();
    Claw claw = new Claw();

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        claw.init(hardwareMap);

        waitForStart();
//        claw.setClawPos(0);
        drive.strafeRight(0.7);

        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < 4000)) {

        }

       //claw.setClawPos(0.5);
        drive.stop();
    }
}

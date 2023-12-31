package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;


@Autonomous(name = "DelayedMoveLeft", group = "bad auto")

public class DelayedMoveLeft extends LinearOpMode{

    private MecanumDrive drive = new MecanumDrive();
    private ElapsedTime runtime = new ElapsedTime();
    private Claw claw = new Claw();

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        claw.init(hardwareMap);

        waitForStart();
        while (opModeIsActive() && (runtime.seconds() < 20)) {

        }
        //claw.setClawPos(0);
        drive.strafeLeft(0.7);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 8)) {

        }

        // claw.setClawPos(0.5);
        drive.stop();
    }

}

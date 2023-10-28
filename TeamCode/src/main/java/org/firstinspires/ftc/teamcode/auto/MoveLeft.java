package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;

@Disabled
@Autonomous(name = "MoveLeft", group = "auto")

public class MoveLeft extends LinearOpMode{

    private MecanumDrive drive = new MecanumDrive();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);

        waitForStart();
        drive.strafeLeft(0.7);

        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < 5000)) {

        }

        drive.stop();
    }

}

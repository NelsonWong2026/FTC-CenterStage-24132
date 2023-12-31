package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Launcher;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;

@TeleOp(name="Main Teleop Mode", group="OpMode")
public class MainTeleop extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive drive = new MecanumDrive();
    private Claw claw = new Claw();
    private Arm arm = new Arm();
    private boolean fieldCentric = false;
    private Launcher launcher = new Launcher();
    /*private Lift lift = new Lift();
    private PIDF_Arm pidf_arm = new PIDF_Arm();*/


    @Override
    public void init() {
        this.drive.init(hardwareMap);
        this.claw.init(hardwareMap);
        this.arm.init(hardwareMap);
        this.launcher.init(hardwareMap);
        /*this.lift.init(hardwareMap);
        this.pidf_arm.init(hardwareMap);*/
        telemetry.addData("Status","Initialized");
        telemetry.update();
        launcher.setLauncherPos(0.8);
    }


    @Override
    public void start() {
        this.runtime.reset();
    }

    @Override
    public void loop() {

        if (this.runtime.seconds() < Constants.Time.teleopTime) {
            this.drive.setControl(gamepad1, fieldCentric);
            this.claw.setControl(gamepad2);
            this.arm.setControl(gamepad2);
            this.launcher.setControl(gamepad2);
            /*this.lift.setControl(gamepad1);
            this.pidf_arm.setControl(gamepad2);*/

            telemetry.addData("Status","Enabled");
            telemetry.addData("Time Remaining",Constants.Time.teleopTime-this.runtime.seconds());
            telemetry.update();
        }

        else {
            this.drive.stop();
        }
    }

    @Override
    public void stop() {
        telemetry.addData("Status","Stopped");
        telemetry.update();
    }
}
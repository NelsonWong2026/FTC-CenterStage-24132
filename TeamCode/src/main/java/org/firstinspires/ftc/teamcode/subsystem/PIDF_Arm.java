package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class PIDF_Arm {
    private PIDController controller;

    private double p = 0, i = 0, d = 0;
    private double f = 0;

    private int target = 0;

    private final double ticks_in_degree = 1680 / 180;

    private DcMotorEx arm_motor;

    public void init(HardwareMap hwMap) {
        controller = new PIDController(p, i, d);

        arm_motor = hwMap.get(DcMotorEx.class, Constants.Arm.Arm);
        this.arm_motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setTarget(int target) {
        this.target = target;
    }

    public void setControl(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            target = 0;
        }
        else if (gamepad.dpad_down) {
            target = 0;
        }

        controller.setPID(p, i, d);
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        arm_motor.setPower(power);
    }
}

package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Arm {
    private DcMotorEx arm;
    private RevTouchSensor touchSensor;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    public Arm() {

    }

    public void init(HardwareMap hwMap) {
        this.arm = hwMap.get(DcMotorEx.class, Constants.Arm.Arm);
        this.arm.setDirection(DcMotor.Direction.FORWARD);
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //this.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // this.touchSensor = hwMap.get(RevTouchSensor.class, Constants.Arm.touchSensor);
    }

   public void zeroCalibrate() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setArmPos(int Position) {
        arm.setTargetPosition(Position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(Constants.Arm.AutoMovePower);
    }

    public void setControl(Gamepad gamepad) {
        Integer armTarget;
        if (gamepad.dpad_down) {
           // armTarget = 4600;
            arm.setPower(1);
        }
        else if (gamepad.dpad_up) {
            //armTarget = 1;
            arm.setPower(-1);
        }
        /*else if (gamepad.dpad_right) {
            armTarget = 2500;
        }*/
        else {
            //armTarget = null;
            arm.setPower(0);
        }
        /*if (armTarget != null) {
            arm.setTargetPosition(armTarget);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(Constants.Arm.MovePower);
        }*/
    }
}

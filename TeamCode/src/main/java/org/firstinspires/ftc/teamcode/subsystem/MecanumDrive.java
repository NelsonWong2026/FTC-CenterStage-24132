package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class MecanumDrive {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    public MecanumDrive() {

    }

    public void init(HardwareMap hwMap) {
        this.leftFront = hwMap.get(DcMotor.class,Constants.MecanumDrive.leftFront);
        this.leftBack = hwMap.get(DcMotor.class, Constants.MecanumDrive.leftBack);
        this.rightFront = hwMap.get(DcMotor.class,Constants.MecanumDrive.rightFront);
        this.rightBack = hwMap.get(DcMotor.class,Constants.MecanumDrive.rightBack);

        this.leftFront.setDirection(DcMotor.Direction.FORWARD);
        this.leftBack.setDirection(DcMotor.Direction.REVERSE);
        this.rightFront.setDirection(DcMotor.Direction.FORWARD);
        this.rightBack.setDirection(DcMotor.Direction.FORWARD);

        this.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    //normal drive
    public void setNormal(double leftPower, double rightPower) {
        this.leftFront.setPower(leftPower);
        this.leftBack.setPower(leftPower);
        this.rightFront.setPower(rightPower);
        this.rightBack.setPower(rightPower);
    }

    //strafe drive
    public void strafeLeft(double Power) {
        this.leftFront.setPower(-Power);
        this.leftBack.setPower(Power);
        this.rightFront.setPower(Power);
        this.rightBack.setPower(-Power);
    }

    public void strafeRight(double Power) {
        this.leftFront.setPower(Power);
        this.leftBack.setPower(-Power);
        this.rightFront.setPower(-Power);
        this.rightBack.setPower(Power);
    }

    public void stop() {
        this.setNormal(0,0);
    }

    public void goForward() {
        this.setNormal(0.5,0.5);
    }

    public void goBackward() {
        this.setNormal(-0.5,-0.5);
    }

    //controller controls
    public void setControl(Gamepad gamepad) {

        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x * 1.1;
        double rx = gamepad.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x -rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

    }
}

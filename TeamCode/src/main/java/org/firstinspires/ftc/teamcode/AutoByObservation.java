package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous (name = "AutoByObservation", group = "teamcode")


public class AutoByObservation extends LinearOpMode {
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor armMotor = null;
    public CRServo intake      = null;
    public Servo wrist       = null;


    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(CRServo.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.FORWARD);



        waitForStart();

        while (opModeIsActive()) {
            moveBack(0.5, 500);
            moveRight(0.5, 3300);
            moveFoward(0.5, 500);
            moveBack(0.5, 1781);
            moveRight(0.5, 500);
            moveFoward(0.5, 1781);
            moveBack(0.5, 1781);
            moveRight(0.5, 500);
            moveFoward(0.5, 1680);
            moveBack(0.5, 650);
            moveLeft(0.5,1050);
            moveLeft(0.5, 4400);
            moveFoward(0.5, 650);
            requestOpModeStop();

        }


    }

    public void moveFoward(double power, long time) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        sleep(time);
        stopMotor();

    }
    public void moveBack(double power, long time) {
        leftFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(-power);
        sleep(time);
        stopMotor();

    }

    public void turnRight(double power, long time) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(-power);

        sleep(time);
        stopMotor();

    }

    public void moveRight(double power, long time) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(-power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(power);
        sleep(time);
        stopMotor();

    }

    public void moveLeft(double power, long time) {
        leftFrontDrive.setPower(-power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(-power);
        sleep(time);
        stopMotor();
    }

    public void turnLeft(double power, long time) {

        leftFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);

        sleep(time);
        stopMotor();

    }

    public void moveArm(double power, long time) {

        armMotor.setPower(power);

        sleep(time);
        stopMotor();


    }

    public void stopMotor(){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        armMotor.setPower(0);
        intake.setPower(0);
    }


}

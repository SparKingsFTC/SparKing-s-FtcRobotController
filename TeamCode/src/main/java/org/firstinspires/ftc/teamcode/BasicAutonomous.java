package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous (name = "BasicAutonomous", group = "teamcode")


public class BasicAutonomous extends LinearOpMode {
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
           //moveArm(0.5, 2000);
           //moveArm(-0.5, 2000);

           stopMotor();
//312 rpm
            //156 rpm
            //20 inches per sec
            //place right next to net zone(NOT IN NEXT)
           //basic autonomos (no way this will work!!)
            // PLACE BACKWARDS
            moveBack(0.5, 1400);
            stopMotor();
            moveRight(0.5, 200);
            stopMotor();
            moveFoward(0.5, 1400);
            stopMotor();
            moveBack(0.5, 1400);
            stopMotor();
            moveRight(0.5, 200);
            stopMotor();
            moveFoward(0.5, 1400);
            stopMotor();
            moveBack(0.5, 1400);
            stopMotor();
            moveRight(0.5, 200);
            stopMotor();
            moveFoward(0.5, 1400);
            stopMotor();
            //captured first 3
            moveBack(0.5, 1400);
            stopMotor();
            moveLeft(0.5, 700);
            stopMotor();
            moveBack(0.5, 1400);
            stopMotor();
            moveRight(0.5, 200);
            stopMotor();
            moveFoward(0.5, 2800);
            stopMotor();
            moveBack(0.5, 2800);
            stopMotor();
            moveRight(0.5, 200);
            stopMotor();
            moveFoward(0.5,2800);
            stopMotor();
            moveBack(0.5, 2800);
            stopMotor();
            moveRight(0.5, 200);
            stopMotor();
            moveFoward(0.5,2800);
            stopMotor();
            moveBack(0.5, 1400);
            //done. probably :)
            stopMotor();

        }

        //2.675 ft per sec


    }

    public void moveFoward(double power, long time) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        sleep(time);

    }
    public void moveBack(double power, long time) {
        leftFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(-power);
        sleep(time);

    }

    public void turnRight(double power, long time) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(-power);

        sleep(time);

    }

    public void moveRight(double power, long time) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(-power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(power);
        sleep(time);

    }

    public void moveLeft(double power, long time) {
        leftFrontDrive.setPower(-power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(-power);
        sleep(time);

    }

    public void turnLeft(double power, long time) {

        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(-power);

        sleep(time);


    }

    public void moveArm(double power, long time) {

        armMotor.setPower(power);

        sleep(time);


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


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
//

            
//312 rpm
            //156 rpm
            //45.01 inches per sec
            //place right next to net zone(NOT IN NEXT)
           //PLACE SO CAN SCORE
            //set wrist at 1
            //10 hole base
            //17 hole arm
            //117 rpm for arm

            //code for arm scoring
            moveFoward(0.5, 298);
            moveArm(0.25, 1372);
            wrist.setPosition(0.5);
            intake.setPower(1);
            intake.setPower(0);
            wrist.setPosition(1);
            moveArm(0.25, -1372);
            moveBack(0.5, 298);

            //code for arm scoring
            moveRight(0.5, 1067);
            moveFoward(0.5, 178);
            moveLeft(0.5, 1067);
            moveRight(0.5, 1067);
            moveFoward(0.5, 223);
            moveLeft(0.5, 1067);
            moveBack(0.5, 401);
            //captured first two. cant capture 3rd because wheel is too thick (maybe come back to pick up with arm?)
            moveBack(0.5, 2133); //96 inches (4 squares)
            moveRight(0.5, 1066); //48 inches (2 squares) r
            moveBack(0.5, 178);
            moveLeft(0.5, 800); // 36 inches (1.5 squares) l
            moveRight(0.5, 267); //12 inches r
            moveBack(0.5, 355);
            moveLeft(0.5, 267);
            moveFoward(0.5, 2311);
            moveFoward(0.5, 355);
            moveBack(0.5, 355);
            //og position
            //got 1, rest cant b/c robot is to big to fit behind samples (cant pick up with arm b/c of placement. need to figure out rotations)
            moveBack(0.5,2133);
            //parked :)

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


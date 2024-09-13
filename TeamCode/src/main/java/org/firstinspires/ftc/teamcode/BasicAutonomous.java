package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "BasicAutonomous", group = "teamcode")


public class BasicAutonomous extends LinearOpMode {
    Robot robot;


    @Override
    public void runOpMode() {

        robot = new Robot();

        waitForStart();

        while (opModeIsActive()){
            moveForward(0.1,1);
            moveForward(0.1,2);
        }



    }
    public void moveForward(double power, long time){

        robot.leftFront.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightFront.setPower(power);
        robot.rightBack.setPower(power);

        sleep(time);

        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
    }
    public void turnRight(double power, long time){

        robot.leftFront.setPower(-power);
        robot.leftBack.setPower(-power);
        robot.rightFront.setPower(power);
        robot.rightBack.setPower(power);

        sleep(time);

        robot.leftFront.setPower(0);
        robot. leftBack.setPower(0);
        robot. rightFront.setPower(0);
        robot.  rightBack.setPower(0);
    }
    public void turnLeft(double power, long time){

        robot.leftFront.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightFront.setPower(-power);
        robot.rightBack.setPower(-power);

        sleep(time);

        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
    }

}

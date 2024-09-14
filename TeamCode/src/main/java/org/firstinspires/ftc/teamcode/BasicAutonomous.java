package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous (name = "BasicAutonomous", group = "teamcode")


public class BasicAutonomous extends LinearOpMode {

    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;


    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBack = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBack = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        while (opModeIsActive()) {
            rightFront.setPower(0.1);
            rightBack.setPower(0.1);
            leftFront.setPower(0.1);
            leftBack.setPower(0.1);
        }


    }
}
 /*  public void moveForward(double power, long time){

       leftFront.setPower(power);
       leftBack.setPower(power);
       rightFront.setPower(power);
        rightBack.setPower(power);

        sleep(time);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
       rightBack.setPower(0);
}*/
//    public void turnRight(double power, long time){
//
//        leftFront.setPower(-power);
////        leftBack.setPower(-power);
////        rightFront.setPower(power);
////        rightBack.setPower(power);
//
//        sleep(time);
//
//        leftFront.setPower(0);
////         leftBack.setPower(0);
////        rightFront.setPower(0);
////        rightBack.setPower(0);
//    }
//    public void turnLeft(double power, long time){
//
//       leftFront.setPower(power);
////        leftBack.setPower(power);
////       rightFront.setPower(-power);
////        rightBack.setPower(-power);
//
//        sleep(time);
//
//        leftFront.setPower(0);
////        leftBack.setPower(0);
////        rightFront.setPower(0);
////        rightBack.setPower(0);
//    }
//
//}

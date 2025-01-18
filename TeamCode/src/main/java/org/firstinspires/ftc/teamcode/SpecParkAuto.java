/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@Autonomous(name="USE To PARK IN auTO anf speiciman", group="Robot")

public class SpecParkAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armMotor = null;
    public DcMotor  liftMotor = null;
    public Servo    wrist = null; //the wrist servo
    public Servo    claw  = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation
    final int ARM_COLLAPSED_INTO_ROBOT  = 10;
    final int ARM_SCORE_SPECIMEN        = (int)(60 * ARM_TICKS_PER_DEGREE);
    final int ARM_SCORE_SPECIMEN2        = (int)(25 * ARM_TICKS_PER_DEGREE);

    final int ARM_WINCH_ROBOT           = (int)(10 * ARM_TICKS_PER_DEGREE);

    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    final double LIFT_COLLAPSED = (int) 0 * LIFT_TICKS_PER_MM;
    final double LIFT_COLLECT =  100 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 580 * LIFT_TICKS_PER_MM;

    double liftPosition = (int) LIFT_COLLAPSED;
    IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the drive system variables.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        wrist  = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//use brake mode for speedy no pauses goofy stuff
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                leftFrontDrive.getCurrentPosition(),
                leftBackDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition(),
                rightBackDrive.getCurrentPosition());
        telemetry.update();
        //STOPPED HERE


        imu.initialize(parameters);

        // Wait for the game to start (driver presses START)
        waitForStart();
        // 45 degree turnLeft(0.7, 720);
        //180 degree turnLeft(0.7, 1370);
//robot is 17 inches
        //never put wrist at 1!! .85 or something
        liftMotorPlacement(0.75, 0);
        claw.setPosition(1);
        ForwardBackward(0.5, 28, -1);
        armMotorPlacement(0.5, ARM_SCORE_SPECIMEN);
        ForwardBackward(0.5,1.5, -1);
        wrist.setPosition(0.67);
        wrist.setPosition(0.9);
        sleep(250);
        quick();

        ForwardBackward(1,4.5, 1);
        armMotorPlacement(0.5, ARM_COLLAPSED_INTO_ROBOT);
        ForwardBackward(0.75, 15, 1);
        armMotorPlacement(0.5, ARM_COLLAPSED_INTO_ROBOT);
        //turnLeft(0.7, 1370);
        turnLeft(0.7, 1415);
        Right(0.75, 55);
        armMotorPlacement(1,0);
        wrist.setPosition(0);
        sleep(1000);
         // pause to display final telemetry message.
        requestOpModeStop();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */

    public void quick(){
        armMotorPlacement(1 ,ARM_SCORE_SPECIMEN2);
        claw.setPosition(0);
        wrist.setPosition(0);
    }
    public void ForwardBackward(double speed, double inches, double movement) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH * movement);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH * movement);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH * movement);
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH * movement);
            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);


            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy() || rightFrontDrive.isBusy())) {

            }


            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // optional pause after each move.
        }
    }


    public void Left(double speed, double inches) {


        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH * -1);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH );
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH * -1);
            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);


            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy() || rightFrontDrive.isBusy())) {

            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void Right(double speed, double inches) {


        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH  * 1);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH * -1 );
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH * -1 );
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH * 1);
            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);


            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy() || rightFrontDrive.isBusy())) {

            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }


    public void liftMotorPlacement(double speed, int liftposition) {


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            liftMotor.setTargetPosition(liftposition);


            // Turn On RUN_TO_POSITION

            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            liftMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (liftMotor.isBusy())) {

            }

            // Stop all motion;

            liftMotor.setPower(0);

            // Turn off RUN_TO_POSITION

            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(500);   // optional pause after each move.
        }
    }


    public void armMotorPlacement(double speed, int armposition) {


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            armMotor.setTargetPosition(armposition);


            // Turn On RUN_TO_POSITION

            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            armMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (armMotor.isBusy())) {

            };

            // Stop all motion;

            armMotor.setPower(0);

            // Turn off RUN_TO_POSITION

            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(500);   // optional pause after each move.
        }


    }
    /* public void turnGyro( double speed, int howmanydegrees){
         YawPitchRollAngles e = imu.getRobotYawPitchRollAngles();
         int robotDegrees = (int) e.getYaw(AngleUnit.DEGREES);
         int degreeTarget =  howmanydegrees;
         while(opModeIsActive()) {
             while (degreeTarget != robotDegrees) {
                 leftFrontDrive.setPower(speed);
                 rightFrontDrive.setPower(-speed);
                 leftBackDrive.setPower(speed);
                 rightBackDrive.setPower(-speed);
             }
         }
         if (degreeTarget == robotDegrees){
             leftFrontDrive.setPower(0);
             rightFrontDrive.setPower(0);
             leftBackDrive.setPower(0);
             rightBackDrive.setPower(0);
         }
     }*/
    public void turnLeft(double power, long time) {
        leftFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        sleep(time);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

}

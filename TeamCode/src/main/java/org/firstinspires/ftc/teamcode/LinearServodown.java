package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name="Linear Servo Testing - down ", group="Linear OpMode")
public class LinearServodown extends LinearOpMode {
    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;
    ServoImplEx servo;
    PwmControl.PwmRange range = new PwmControl.PwmRange(900, 2100);
    @Override
    public void runOpMode() {
        servo = hardwareMap.get(ServoImplEx.class, "linearservo");
        servo.setPwmRange(range);
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            servo.setPosition(MIN_POS);
            sleep(20000);
        }
    }
}
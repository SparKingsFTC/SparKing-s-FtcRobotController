package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Linear Servo Testing", group="Linear OpMode")
public class LinearServo extends LinearOpMode {
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
            servo.setPosition(MAX_POS);
            sleep(4000);
            servo.setPosition(MIN_POS);
            sleep(4000);
        }
    }
}
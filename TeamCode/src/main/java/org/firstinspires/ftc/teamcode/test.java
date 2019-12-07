package org.firstinspires.ftc.teamcode;
import android.sax.TextElementListener;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * Created by robotics on 10/30/2017.
 */
@TeleOp(name = "Teleop", group = "TeleOp")
public class test extends LinearOpMode {

    private DcMotor a;
    private DcMotor b;
    private DcMotor driveBackLeft;
    private DcMotor driveBackRight;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    private DistanceSensor distance;
    @Override
    public void runOpMode() throws InterruptedException {

       a = hardwareMap.dcMotor.get("a");
        b = hardwareMap.dcMotor.get("b");



        a.setDirection(DcMotor.Direction.REVERSE);


        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (opModeIsActive()) {

            a.setPower(gamepad1.left_stick_y);
            b.setPower(gamepad1.left_stick_y);


            //    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            //else
              //  blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
            // GAMEPAD 1 BASE

            //driveBackLeft.setPower(gamepad1.left_stick_y);
            //driveFrontLeft.setPower(gamepad1.left_stick_y);
            //driveFrontRight.setPower(gamepad1.left_stick_y);
            //driveBackRight.setPower(gamepad1.left_stick_y);


        }
        idle();
    }
}
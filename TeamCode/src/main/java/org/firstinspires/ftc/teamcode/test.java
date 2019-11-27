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

    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveBackLeft;
    private DcMotor driveBackRight;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    private DistanceSensor distance;
    @Override
    public void runOpMode() throws InterruptedException {

        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");

        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);


        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (opModeIsActive()) {
            driveBackLeft.setPower(0);
            driveBackRight.setPower(0);
            driveFrontLeft.setPower(0);
            driveFrontRight.setPower(0);

            //    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            //else
              //  blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
            // GAMEPAD 1 BASE

            //driveBackLeft.setPower(gamepad1.left_stick_y);
            //driveFrontLeft.setPower(gamepad1.left_stick_y);
            //driveFrontRight.setPower(gamepad1.left_stick_y);
            //driveBackRight.setPower(gamepad1.left_stick_y);
            driveBackLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
            driveFrontLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x*-1);
            driveFrontRight.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
            driveBackRight.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x*-1);
            // RIGHT STICK X - TURN CLOCKWISE AND COUNTERCLOCKWISE
              driveFrontLeft.setPower(gamepad1.right_stick_x * -1);
              driveBackLeft.setPower(gamepad1.right_stick_x * -1);
              driveFrontRight.setPower(gamepad1.right_stick_x);
              driveBackRight.setPower(gamepad1.right_stick_x);

            driveBackLeft.setPower( gamepad1.right_stick_x);
            driveFrontLeft.setPower( gamepad1.right_stick_x*-1);
            driveFrontRight.setPower( gamepad1.right_stick_x);
            driveBackRight.setPower(gamepad1.right_stick_x*-1);

        }
        idle();
    }
}
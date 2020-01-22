package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp_Rylan", group = "TeleOp")

public class TeleOp_Rylan extends LinearOpMode {

    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveBackLeft;
    private DcMotor driveBackRight;
    private Servo rotation;
    private Servo clamp;

    private DcMotor liftleft;
    private DcMotor liftright;
    private DcMotor actuator;
    DigitalChannel limitSwitch;
    public Servo rightFoundation;
    public Servo leftFoundation;
    double toggle;
    boolean toggler;

    @Override
    public void runOpMode() throws InterruptedException {

        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");
        rotation = hardwareMap.servo.get("rotation");
        actuator = hardwareMap.dcMotor.get("actuator");
        clamp = hardwareMap.servo.get("clamp");
        liftright = hardwareMap.dcMotor.get("liftright");
        liftleft = hardwareMap.dcMotor.get("liftleft");

        driveFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        liftright.setDirection(DcMotor.Direction.REVERSE);

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        rightFoundation = hardwareMap.servo.get("rightFoundation");
        leftFoundation = hardwareMap.servo.get("leftFoundation");

        // set the digital channel to input.
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        toggle = 0.6;

        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (opModeIsActive()) {

            //Gamepad 1 left joystick x strafe
            while ((Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) && gamepad1.left_bumper && opModeIsActive()) {
                driveBackLeft.setPower(gamepad1.left_stick_y * 0.3 + gamepad1.left_stick_x * 0.3 + gamepad1.right_stick_x * -0.3);
                driveFrontLeft.setPower(gamepad1.left_stick_y * 0.3 + gamepad1.left_stick_x * -0.3 + gamepad1.right_stick_x * -0.3);
                driveFrontRight.setPower(gamepad1.left_stick_y * 0.3 + gamepad1.left_stick_x * 0.3 + gamepad1.right_stick_x * 0.3);
                driveBackRight.setPower(gamepad1.left_stick_y * 0.3 + gamepad1.left_stick_x * -0.3 + gamepad1.right_stick_x * 0.3);
            }
            //Gamepad 1 left joystick x strafe
            while ((Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) && !gamepad1.left_bumper && opModeIsActive()) {
                driveBackLeft.setPower(gamepad1.left_stick_y * 0.6 + gamepad1.left_stick_x * 0.6 + gamepad1.right_stick_x * -0.6);
                driveFrontLeft.setPower(gamepad1.left_stick_y * 0.6 + gamepad1.left_stick_x * -0.6 + gamepad1.right_stick_x * -0.6);
                driveFrontRight.setPower(gamepad1.left_stick_y * 0.6 + gamepad1.left_stick_x * 0.6 + gamepad1.right_stick_x * 0.6);
                driveBackRight.setPower(gamepad1.left_stick_y * 0.6 + gamepad1.left_stick_x * -0.6 + gamepad1.right_stick_x * 0.6);
            }

            driveBackLeft.setPower(0);
            driveFrontLeft.setPower(0);
            driveFrontRight.setPower(0);
            driveBackRight.setPower(0);

            if (gamepad2.right_bumper && opModeIsActive()) {
                clamp.setPosition(.25);
            }
            if (gamepad2.left_bumper && opModeIsActive()) {
                clamp.setPosition(1);
            }

            actuator.setPower(gamepad2.right_trigger);
            actuator.setPower(gamepad2.left_trigger * -1);

            if (gamepad2.dpad_left && opModeIsActive()) {
                rotation.setPosition(.98);
            }
            if (gamepad2.dpad_right && opModeIsActive()) {
                rotation.setPosition(.64);
            }


            if (gamepad2.left_stick_y < 0 && opModeIsActive() && limitSwitch.getState()) {
                liftleft.setPower(gamepad2.left_stick_y * -0.05);
                liftright.setPower(gamepad2.left_stick_y * -0.05);
                telemetry.addLine("DOWN");
                telemetry.update();
            } else if (gamepad2.left_stick_y > 0 && opModeIsActive()) {
                liftleft.setPower(gamepad2.left_stick_y * -0.65);
                liftright.setPower(gamepad2.left_stick_y * -0.65);
                telemetry.addLine("UP");
                telemetry.update();
            } else if (gamepad2.left_stick_y == 0 && opModeIsActive() && !limitSwitch.getState()) {
                liftright.setPower(0);
                liftleft.setPower(0);
                telemetry.addLine("LIMIT SWITCH PRESSED");
                telemetry.update();
            } else if (gamepad2.left_stick_y == 0 && opModeIsActive() && limitSwitch.getState()) {
                liftright.setPower(-0.18);
                liftleft.setPower(-0.18);
                telemetry.addLine("HOLD");
                telemetry.update();
            }
            if (gamepad2.x && opModeIsActive()) {
                leftFoundation.setPosition(1);
                rightFoundation.setPosition(0.15);
            } else {
                leftFoundation.setPosition(0.2);
                rightFoundation.setPosition(.9);
            }
            idle();
        }
    }
}

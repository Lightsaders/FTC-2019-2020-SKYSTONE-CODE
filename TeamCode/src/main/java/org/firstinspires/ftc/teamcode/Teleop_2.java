package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class Teleop_2 extends LinearOpMode {

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
    public boolean toggle;

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

        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (opModeIsActive()) {

            leftFoundation.setPosition(0.2);
            rightFoundation.setPosition(1);

            //Gamepad 1 left joystick x strafe
            while(Math.abs(gamepad1.left_stick_x) > 0.1 && opModeIsActive()) {
                driveFrontLeft.setPower(gamepad1.left_stick_x*-0.75);
                driveBackLeft.setPower(gamepad1.left_stick_x * 0.75);
                driveFrontRight.setPower(gamepad1.left_stick_x * 0.75);
                driveBackRight.setPower(gamepad1.left_stick_x*-0.75);
            }

            //Gamepad 1 right joystick y straight
            while(Math.abs(gamepad1.right_stick_y) > 0.1&& opModeIsActive()) {
                driveBackLeft.setPower(gamepad1.right_stick_y*0.75);
                driveFrontLeft.setPower(gamepad1.right_stick_y*0.75);
                driveFrontRight.setPower(gamepad1.right_stick_y*0.75);
                driveBackRight.setPower(gamepad1.right_stick_y*0.75);
            }

            // Gamepad 1 triggers for turning
            while(Math.abs(gamepad1.right_trigger) > 0.1&& opModeIsActive()) {
                driveBackLeft.setPower(gamepad1.right_trigger * -0.5);
                driveFrontLeft.setPower(gamepad1.right_trigger * -0.5);
                driveFrontRight.setPower(gamepad1.right_trigger*0.5);
                driveBackRight.setPower(gamepad1.right_trigger*0.5);
            }

            while(Math.abs(gamepad1.left_trigger) > 0.1&& opModeIsActive()) {
                driveBackLeft.setPower(gamepad1.left_trigger*0.5);
                driveFrontLeft.setPower(gamepad1.left_trigger*0.5);
                driveFrontRight.setPower(gamepad1.left_trigger *-0.5);
                driveBackRight.setPower(gamepad1.left_trigger *-0.5);
            }

            driveBackLeft.setPower(0);
            driveFrontLeft.setPower(0);
            driveFrontRight.setPower(0);
            driveBackRight.setPower(0);

            if (gamepad2.right_bumper&& opModeIsActive()) {
                clamp.setPosition(.45);
            }
            if (gamepad2.left_bumper&& opModeIsActive()) {
                clamp.setPosition(1);
            }

            actuator.setPower(gamepad2.right_trigger);
            actuator.setPower(gamepad2.left_trigger * -1);

            if (gamepad2.dpad_left&& opModeIsActive()) {
                rotation.setPosition(.4);
            }
            if (gamepad2.dpad_right&& opModeIsActive()) {
                rotation.setPosition(.8);
            }

            if (gamepad2.left_stick_y < 0 && opModeIsActive() && limitSwitch.getState()) {
                liftleft.setPower(gamepad2.left_stick_y*-0.4);
                liftright.setPower(gamepad2.left_stick_y*-0.4);
                telemetry.addLine("DOWN");
                telemetry.update();
            } else
            if (gamepad2.left_stick_y > 0 && opModeIsActive()) {
                liftleft.setPower(gamepad2.left_stick_y*-0.7);
                liftright.setPower(gamepad2.left_stick_y*-0.7);
                telemetry.addLine("UP");
                telemetry.update();
            }else
            if (gamepad2.left_stick_y == 0 && opModeIsActive()&& !limitSwitch.getState()) {
                liftright.setPower(0);
                liftleft.setPower(0);
                telemetry.addLine("LIMIT SWITCH PRESSED");
                telemetry.update();
            }else
            if(gamepad2.left_stick_y == 0 && opModeIsActive()&& limitSwitch.getState()) {
                liftright.setPower(-0.2);
                liftleft.setPower(-0.2);
                telemetry.addLine("HOLD");
                telemetry.update();
            }
        }
        idle();
    }
}

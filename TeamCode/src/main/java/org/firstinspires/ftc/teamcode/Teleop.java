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

@TeleOp(name = "Teleop", group = "TeleOp")
public class Teleop extends LinearOpMode {

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

            leftFoundation.setPosition(0);
            rightFoundation.setPosition(1);

            //    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            //else
            //  blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
            // GAMEPAD 1 BASE
            telemetry.addData("Left stick y",gamepad1.left_stick_y);
            telemetry.addData("Right stick x",gamepad1.right_stick_x);
            telemetry.addData("Left stick x",gamepad1.left_stick_x);
            telemetry.addData("Right stick y",gamepad1.right_stick_y);
            telemetry.update();
            driveBackLeft.setPower(gamepad1.right_stick_y);
            driveFrontLeft.setPower(gamepad1.right_stick_y);
            driveFrontRight.setPower(gamepad1.right_stick_y);
            driveBackRight.setPower(gamepad1.right_stick_y);
//            driveBackLeft.setPower(gamepad1.left_stick_x*-1);
//            driveFrontLeft.setPower(gamepad1.left_stick_x*-1);
//            driveFrontRight.setPower(gamepad1.left_stick_x);
//            driveBackRight.setPower(gamepad1.left_stick_x);
////            driveBackLeft.setPower(gamepad1.left_stick_y *.8 + gamepad1.left_stick_x*.8);
////            driveFrontLeft.setPower(gamepad1.left_stick_y *.8 + gamepad1.left_stick_x * -.8);
////            driveFrontRight.setPower(gamepad1.left_stick_y*.8 + gamepad1.left_stick_x*.8);
////            driveBackRight.setPower(gamepad1.left_stick_y*.8 + gamepad1.left_stick_x * -.8);
////            // RIGHT STICK X - TURN CLOCKWISE AND COUNTERCLOCKWISE
////            driveFrontLeft.setPower(gamepad1.right_stick_x * -.8);
////            driveBackLeft.setPower(gamepad1.right_stick_x * -.8);
////            driveFrontRight.setPower(gamepad1.right_stick_x*.8);
////            driveBackRight.setPower(gamepad1.right_stick_x*.8);
//
//
//            //TODO make it not have to be held
//
////            while (gamepad1.right_bumper) {
////            driveBackLeft.setPower(gamepad1.left_trigger);
////            driveFrontLeft.setPower(gamepad1.left_trigger);
////            driveFrontRight.setPower(gamepad1.left_trigger);
////            driveBackRight.setPower(gamepad1.left_trigger);
//            // RIGHT STICK X - TURN CLOCKWISE AND COUNTERCLOCKWISE
            driveFrontLeft.setPower(gamepad1.right_stick_x);
            driveBackLeft.setPower(gamepad1.right_stick_x * -1);
            driveFrontRight.setPower(gamepad1.right_stick_x * -1);
            driveBackRight.setPower(gamepad1.right_stick_x);
////            }

//            if(gamepad1.a){
//                driveBackLeft.setPower(.3);
//                driveFrontLeft.setPower(.3);
//                driveBackRight.setPower(.3);
//                driveFrontRight.setPower(.3);
//            }
//            driveBackLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x * -1);
//            driveFrontLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x );
//            driveFrontRight.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x*-1);
//            driveBackRight.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
//            // RIGHT STICK X - TURN CLOCKWISE AND COUNTERCLOCKWISE
//            driveFrontLeft.setPower(gamepad1.right_stick_x * -1);
//            driveBackLeft.setPower(gamepad1.right_stick_x * -1);
//            driveFrontRight.setPower(gamepad1.right_stick_x);
//            driveBackRight.setPower(gamepad1.right_stick_x);

            if (gamepad2.right_bumper) {
                clamp.setPosition(.68);
            }
            if (gamepad2.left_bumper) {
                clamp.setPosition(1);
            }

//            if(gamepad2.b){// forward
//                intake.setPower(1.0);
//                liftleft.setMode(DcMotor.RunMode.RESET_ENCODERS);
//                liftright.setMode(DcMotor.RunMode.RESET_ENCODERS);
//                liftleft.getCurrentPosition();
//                liftright.getCurrentPosition();
//                liftleft.setTargetPosition(20);
//                liftright.setTargetPosition(20);
//
//                // Turn on run to position
//                liftleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftleft.setPower(.5);
//                liftright.setPower(.5);
//                sleep(1000);
//                liftleft.setPower(0);
//                liftright.setPower(0);
//
//                //Turn off run to position
//
//                liftleft.setPower(0);
//                liftright.setPower(0);
//                //Turn off run to position
//                liftleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                liftright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            }
            actuator.setPower(gamepad2.right_trigger);
            actuator.setPower(gamepad2.left_trigger * -1);

            while (gamepad2.dpad_left) {
                rotation.setPosition(.4);
            }
            while (gamepad2.dpad_right) {
                rotation.setPosition(.8);
            }
            if (gamepad2.left_stick_y < 0 && limitSwitch.getState() == true) {
                liftleft.setPower(gamepad2.left_stick_y);
                liftright.setPower(gamepad2.left_stick_y);
            } else if (gamepad2.left_stick_y > 0) {
                liftleft.setPower(gamepad2.left_stick_y);
                liftright.setPower(gamepad2.left_stick_y);
            }else if(limitSwitch.getState() == false) {
                liftright.setPower(0);
                liftleft.setPower(0);
            }else if(toggle == true){
                liftright.setPower(0.16);
                liftleft.setPower(0.16);
            }else if(toggle == false){
                liftright.setPower(0);
                liftleft.setPower(0);
            }
            if (gamepad2.y){
                 toggle = !toggle;
            }




        }
        idle();
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Teleop", group = "TeleOp")
public class testnewboy extends LinearOpMode {

    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveBackLeft;
    private DcMotor driveBackRight;
    private Servo Rotation;
    private Servo clamp;

    private DcMotor liftleft;
    private DcMotor liftright;
    private DcMotor actuator;

    @Override
    public void runOpMode() throws InterruptedException {

        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");
        Rotation = hardwareMap.servo.get("Rotation");
        actuator = hardwareMap.dcMotor.get("actuator");
        clamp = hardwareMap.servo.get("clamp");

        liftright = hardwareMap.dcMotor.get("liftright");
        liftleft = hardwareMap.dcMotor.get("liftleft");
        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);
        liftleft.setDirection(DcMotor.Direction.REVERSE);

        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (opModeIsActive()) {


            //    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            //else
            //  blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
            // GAMEPAD 1 BASE

            //driveBackLeft.setPower(gamepad1.left_stick_y);
            //driveFrontLeft.setPower(gamepad1.left_stick_y);
            //driveFrontRight.setPower(gamepad1.left_stick_y);
            //driveBackRight.setPower(gamepad1.left_stick_y);
//            driveBackLeft.setPower(gamepad1.left_stick_y *.8 + gamepad1.left_stick_x*.8);
//            driveFrontLeft.setPower(gamepad1.left_stick_y *.8 + gamepad1.left_stick_x * -.8);
//            driveFrontRight.setPower(gamepad1.left_stick_y*.8 + gamepad1.left_stick_x*.8);
//            driveBackRight.setPower(gamepad1.left_stick_y*.8 + gamepad1.left_stick_x * -.8);
//            // RIGHT STICK X - TURN CLOCKWISE AND COUNTERCLOCKWISE
//            driveFrontLeft.setPower(gamepad1.right_stick_x * -.8);
//            driveBackLeft.setPower(gamepad1.right_stick_x * -.8);
//            driveFrontRight.setPower(gamepad1.right_stick_x*.8);
//            driveBackRight.setPower(gamepad1.right_stick_x*.8);


            //TODO make it not have to be held
            while (gamepad1.right_bumper){
                telemetry.addLine("yay it works");
                telemetry.update();
//                driveBackLeft.setPower(gamepad1.left_stick_y *.24 + gamepad1.left_stick_x*.24);
//                driveFrontLeft.setPower(gamepad1.left_stick_y *.24 + gamepad1.left_stick_x * -.24);
//                driveFrontRight.setPower(gamepad1.left_stick_y*.24 + gamepad1.left_stick_x*.24);
//                driveBackRight.setPower(gamepad1.left_stick_y*.24 + gamepad1.left_stick_x * -.24);
//                // RIGHT STICK X - TURN CLOCKWISE AND COUNTERCLOCKWISE
//                driveFrontLeft.setPower(gamepad1.right_stick_x * -.24);
//                driveBackLeft.setPower(gamepad1.right_stick_x * -.24);
//                driveFrontRight.setPower(gamepad1.right_stick_x*.24);
//                driveBackRight.setPower(gamepad1.right_stick_x*.24);
            }
            while(gamepad1.right_bumper){
                driveBackLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
                driveFrontLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x*-1);
                driveFrontRight.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
                driveBackRight.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x*-1);
                // RIGHT STICK X - TURN CLOCKWISE AND COUNTERCLOCKWISE
                driveFrontLeft.setPower(gamepad1.right_stick_x * -1);
                driveBackLeft.setPower(gamepad1.right_stick_x * -1);
                driveFrontRight.setPower(gamepad1.right_stick_x);
                driveBackRight.setPower(gamepad1.right_stick_x);
            }
            while(gamepad1.left_bumper){
                driveBackLeft.setPower(gamepad1.left_stick_y *.3 + gamepad1.left_stick_x*.3);
                driveFrontLeft.setPower(gamepad1.left_stick_y*.3+ gamepad1.left_stick_x*-.3);
                driveFrontRight.setPower(gamepad1.left_stick_y*.3 + gamepad1.left_stick_x*.3);
                driveBackRight.setPower(gamepad1.left_stick_y*.3 + gamepad1.left_stick_x*-.3);
                // RIGHT STICK X - TURN CLOCKWISE AND COUNTERCLOCKWISE
                driveFrontLeft.setPower(gamepad1.right_stick_x * -.3);
                driveBackLeft.setPower(gamepad1.right_stick_x * -.3);
                driveFrontRight.setPower(gamepad1.right_stick_x*.3);
                driveBackRight.setPower(gamepad1.right_stick_x*.3);
            }
           
            driveBackLeft.setPower(gamepad1.left_stick_y *.8 + gamepad1.left_stick_x*.8);
            driveFrontLeft.setPower(gamepad1.left_stick_y*.8+ gamepad1.left_stick_x*-.8);
            driveFrontRight.setPower(gamepad1.left_stick_y*.8 + gamepad1.left_stick_x*.8);
            driveBackRight.setPower(gamepad1.left_stick_y*.8 + gamepad1.left_stick_x*-.8);
            // RIGHT STICK X - TURN CLOCKWISE AND COUNTERCLOCKWISE
            driveFrontLeft.setPower(gamepad1.right_stick_x * -.8);
            driveBackLeft.setPower(gamepad1.right_stick_x * -.8);
            driveFrontRight.setPower(gamepad1.right_stick_x*.8);
            driveBackRight.setPower(gamepad1.right_stick_x*.8);
            if (gamepad2.right_bumper){
                clamp.setPosition(.68);
            }
            if (gamepad2.left_bumper){
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
            actuator.setPower(gamepad2.left_trigger*-1);

            while (gamepad2.dpad_left){
                Rotation.setPosition(.4);
            }
            while (gamepad2.dpad_right){
                Rotation.setPosition(.8);
            }
            liftleft.setPower(gamepad2.left_stick_y);
            liftright.setPower(gamepad2.left_stick_y);
            while (gamepad2.y){
                liftleft.setPower(.16);
                liftright.setPower(.16);
            }




        }
        idle();
    }
}

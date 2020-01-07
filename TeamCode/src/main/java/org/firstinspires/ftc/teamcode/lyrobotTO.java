package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Teleoply", group = "TeleOp")
public class lyrobotTO extends LinearOpMode {

    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveBackLeft;
    private DcMotor driveBackRight;

    @Override
    public void runOpMode() throws InterruptedException {

        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");

        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);


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

                telemetry.addLine("yay it works");
                telemetry.update();
//

                driveBackLeft.setPower(gamepad1.left_stick_y * .8 + gamepad1.left_stick_x * .8);
                driveFrontLeft.setPower(gamepad1.left_stick_y * .8 + gamepad1.left_stick_x * -.8);
                driveFrontRight.setPower(gamepad1.left_stick_y * .8 + gamepad1.left_stick_x * .8);
                driveBackRight.setPower(gamepad1.left_stick_y * .8 + gamepad1.left_stick_x * -.8);
                // RIGHT STICK X - TURN CLOCKWISE AND COUNTERCLOCKWISE
                driveFrontLeft.setPower(gamepad1.right_stick_x * -.8);
                driveBackLeft.setPower(gamepad1.right_stick_x * -.8);
                driveFrontRight.setPower(gamepad1.right_stick_x * .8);
                driveBackRight.setPower(gamepad1.right_stick_x * .8);



            idle();
        }
    }
}

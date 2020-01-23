package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Last_Year_Teleop", group = "TeleOp")
public class Last_Year_Teleop extends LinearOpMode {

    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveBackLeft;
    private DcMotor driveBackRight;
    private double toggle;

    @Override
    public void runOpMode() throws InterruptedException {

        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");

        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);

        driveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        toggle = 0.8;

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
//            driveBackLeft.setPower(gamepad1.left_stick_y *1 + gamepad1.left_stick_x*1);
//            driveFrontLeft.setPower(gamepad1.left_stick_y *1 + gamepad1.left_stick_x * -1);
//            driveFrontRight.setPower(gamepad1.left_stick_y*1 + gamepad1.left_stick_x*1);
//            driveBackRight.setPower(gamepad1.left_stick_y*1 + gamepad1.left_stick_x * -1);
//            // RIGHT STICK X - TURN CLOCKWISE AND COUNTERCLOCKWISE
//            driveFrontLeft.setPower(gamepad1.right_stick_x * -1);
//            driveBackLeft.setPower(gamepad1.right_stick_x * -1);
//            driveFrontRight.setPower(gamepad1.right_stick_x*1);
//            driveBackRight.setPower(gamepad1.right_stick_x*1);


            //TODO make it not have to be held
//

                driveBackLeft.setPower(gamepad1.left_stick_y * -toggle + gamepad1.left_stick_x * toggle);
                driveFrontLeft.setPower(gamepad1.left_stick_y * toggle + gamepad1.left_stick_x * toggle);
                driveFrontRight.setPower(gamepad1.left_stick_y * -toggle + gamepad1.left_stick_x * toggle);
                driveBackRight.setPower(gamepad1.left_stick_y * toggle + gamepad1.left_stick_x * toggle);
                // RIGHT STICK X - TURN CLOCKWISE AND COUNTERCLOCKWISE
                driveFrontLeft.setPower(gamepad1.right_stick_x * -toggle);
                driveBackLeft.setPower(gamepad1.right_stick_x * -toggle);
                driveFrontRight.setPower(gamepad1.right_stick_x * toggle);
                driveBackRight.setPower(gamepad1.right_stick_x * toggle);
                
                if(gamepad1.x){
                    toggle = 1;
                }
                if(gamepad1.a){
                    toggle = 0.8;
                }
                if(gamepad1.b){
                    toggle = 0.5;
                }

                telemetry.addData("SPEED", + toggle);
                telemetry.update();



            idle();
        }
    }
}

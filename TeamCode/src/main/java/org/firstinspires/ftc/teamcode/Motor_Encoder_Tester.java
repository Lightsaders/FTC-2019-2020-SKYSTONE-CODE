package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous (name = "Encoder Tester")
@Disabled
public class Motor_Encoder_Tester extends LinearOpMode {
    // Drive Motors
    public DcMotor driveFrontLeft;
    public DcMotor driveFrontRight;
    public DcMotor driveBackLeft;
    public DcMotor driveBackRight;
    @Override
    public void runOpMode() throws InterruptedException {
        // Init Drive Motors
        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");

        driveFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            driveFrontLeft.setPower(1);
            driveFrontRight.setPower(1);
            driveBackLeft.setPower(1);
            driveBackRight.setPower(1);

            telemetry.addData("FRONT LEFT MOTOR", driveFrontLeft.getCurrentPosition());
            telemetry.addData("FRONT RIGHT MOTOR", driveFrontRight.getCurrentPosition());
            telemetry.addData("BACK LEFT MOTOR", driveBackLeft.getCurrentPosition());
            telemetry.addData("BACK RIGHT MOTOR", driveBackRight.getCurrentPosition());
            telemetry.update();
        }
    }
}

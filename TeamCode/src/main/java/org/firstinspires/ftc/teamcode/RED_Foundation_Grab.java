package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RED_FOUNDATION_GRAB")
public class RED_Foundation_Grab extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        // init robot
        initialize();
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            rightFoundation.setPosition(.4);
            sleep(1000);
            actuator.setPower(1);//TODO use method created however it requires encoders on actuator
            sleep(900);
            actuator.setPower(0);
//            sleep(1800);
            turnClamp("PAR", 700);
            clamp("OPEN", 500);
            actuator.setPower(-1);//TODO use method created however it requires encoders on actuator
            rotation.setPosition(.98);
            sleep(500);
            actuator.setPower(0);

            clamp.setPosition(.8);
            strafeDriveEncoder(0.3, 39,  "RIGHT",1.5);


            gyroDrive(.2, 88, 0);
            leftFoundation.setPosition(0.9);
            rightFoundation.setPosition(0.15);
            sleep(1600);
            gyroDrive(.3, -94, 0);
            leftFoundation.setPosition(0.2);
            rightFoundation.setPosition(1);
            sleep(1500);
            gyroStrafe(0.3, 118, 0, "LEFT");
            clamp("CLOSE", 700);
        }
    }
}

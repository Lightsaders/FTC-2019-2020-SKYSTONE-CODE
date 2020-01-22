package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUE_FOUNDATION_GRAB")
public class BLUE_Foundation_Grab extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        // init robot
        initCompBot();
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            gyroStrafe(0.3, 39, 0, "LEFT",2);
            actuator.setPower(1);
            sleep(100);
            actuator.setPower(0);
            turnClamp("PAR", 700);
            clamp("OPEN", 500);
            gyroDrive(.2, 88, 0,2);
            leftFoundation.setPosition(0.9);
            rightFoundation.setPosition(0.15);
            sleep(1600);
            gyroDrive(.3, -100, 0,2);
            leftFoundation.setPosition(0.2);
            rightFoundation.setPosition(1);
            sleep(1500);
            gyroStrafe(0.3, 112, 0, "RIGHT",2);// TODO REPLACE WITH NORMAL STRAFE DRIVE
            clamp("CLOSE", 700);
        }
    }
}

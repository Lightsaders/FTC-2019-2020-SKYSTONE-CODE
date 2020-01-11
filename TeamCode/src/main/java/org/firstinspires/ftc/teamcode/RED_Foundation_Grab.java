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

            strafeDriveEncoder(.7,29,"RIGHT",.75);
            straightDriveEncoder(.3,120,2);
            leftFoundation.setPosition(0.9);
            rightFoundation.setPosition(0.15);
            sleep(1500);
            straightDriveEncoder(.4,-160,2);
            leftFoundation.setPosition(0.2);
            rightFoundation.setPosition(1);
            sleep(1500);
            strafeDriveEncoder(.5,110,"LEFT",3.5);
        }
    }
}

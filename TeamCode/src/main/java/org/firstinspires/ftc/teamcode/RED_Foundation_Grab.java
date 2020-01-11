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

            strafeDriveEncoder(.7,19,"RIGHT",.75);
            straightDriveEncoder(.3,120,2);
            foundationClamps("DOWN",1400);
            straightDriveEncoder(.3,-130,2);
            foundationClamps("UP",1400);
            strafeDriveEncoder(.5,100,"LEFT",3.5);
        }
    }
}

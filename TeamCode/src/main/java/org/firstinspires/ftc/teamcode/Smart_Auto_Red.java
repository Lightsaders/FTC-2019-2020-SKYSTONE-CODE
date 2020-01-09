package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous (name = "SMART_AUTO_RED")
public class Smart_Auto_Red extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        // init robot
        initialize();
        telemetry.addLine();
        telemetry.update();
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            // Robot actions in auto

            straightDriveEncoder(1,20);

            strafeDriveEncoder(1,20,"Left");// TODO Need to test this and make sure it works

            turnEncoder(1,45,"Left");// TODO Need to test this and make sure it works

            skystoneColorScan();// TODO Need to test this and make sure it works

        }
    }
}

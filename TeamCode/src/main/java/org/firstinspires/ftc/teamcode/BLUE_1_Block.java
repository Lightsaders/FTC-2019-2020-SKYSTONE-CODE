package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "BLUE_1_Block")
@Disabled
public class BLUE_1_Block extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        // init robot
        initialize();
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            turnClamp("PAR", 250);
            clamp("OPEN", 250);
            straightDriveEncoder(0.2, 106, 3);
            skystoneColorScan("BLUE");
            telemetry.addLine("Skystone position: " + positionSkystone);
            telemetry.update();
            switch (positionSkystone) {
                case "WALL":
                    if (!isStopRequested() && opModeIsActive()) {
                        straightDriveEncoder(.2, -9, 0);
                        strafeDriveEncoder(1, 15, "RIGHT",3);
                        actuator.setPower(1);//TODO use method created
                        sleep(400);
                        actuator.setPower(0);
                        clamp("CLOSE", 250);
                        straightDriveEncoder(0.2, -30, 1);
                        turnEncoder(.4, 95, "CC",3);
                        straightDriveEncoder(0.6, 200, 1);
                        clamp("OPEN", 250);
                        straightDriveEncoder(.5, -65, 0);
                    }
                    break;
                case "MIDDLE":
                    if (!isStopRequested() && opModeIsActive()) {
                        strafeDriveEncoder(1, 15, "LEFT",3);
                        actuator.setPower(1);//TODO use method created
                        sleep(300);
                        actuator.setPower(0);
                        clamp("CLOSE", 250);
                        straightDriveEncoder(0.2, -30, 1);
                        turnEncoder(.4, 95, "CC",3);
                        straightDriveEncoder(0.6, 170, 0);
                        clamp("OPEN", 250);
                        straightDriveEncoder(.5, -65, 0);
                    }
                    break;
                case "BRIDGE":
                    if (!isStopRequested() && opModeIsActive()) {
                        straightDriveEncoder(.2, -9, 0);
                        strafeDriveEncoder(1, 35, "LEFT",3);
                        actuator.setPower(1);//TODO use method created
                        sleep(300);
                        actuator.setPower(0);
                        clamp("CLOSE", 250);
                        straightDriveEncoder(0.2, -30, 1.5);
                        turnEncoder(.4, 95, "CC",3);
                        straightDriveEncoder(0.2, 150, 3);
                        clamp("OPEN", 250);
                        straightDriveEncoder(.5, -40, 0);
                    }
                    break;
            }
        }
    }
}
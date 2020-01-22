package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RED_2_BLOCK")
public class RED_2_Block extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        // init robot
        initialize();
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            rightFoundation.setPosition(.4);
            sleep(1000);
            actuator.setPower(1);//TODO use method created however it requires encoders on actuator
            sleep(300);
            actuator.setPower(0);
            turnClamp("PAR", 700);
            clamp("OPEN", 500);
            actuator.setPower(-1);//TODO use method created however it requires encoders on actuator
            sleep(100);
            actuator.setPower(0);
            turnClamp("PAR", 250);
            clamp("OPEN", 250);
            straightDriveEncoder(0.2, 95, 2.5);// TODO adjust tomorrow morning
            skystoneColorScan("RED");
            telemetry.addLine("Skystone position: " + positionSkystone);
            telemetry.update();
            switch (positionSkystone) {
                case "WALL":
                    if (!isStopRequested() && opModeIsActive()) {
                        straightDriveEncoder(0.2, -9, 0.75);
                        strafeDriveEncoder(1, 10, "LEFT", 0.75);
                        actuator.setPower(1);//TODO use method created however it requires encoders on actuator
                        sleep(500);
                        actuator.setPower(0);
                        clamp("CLOSE", 250);
                        straightDriveEncoder(0.6, -30, 0.75);
                        turnEncoder(0.5, 88, "C", 1);
                        straightDriveEncoder(0.7, 190, 1.75);
                        clamp("OPEN", 250);
                        straightDriveEncoder(0.7, -217, 2);
                        turnEncoder(0.5, 168, "C", 1.75);
                        strafeDriveEncoder(0.5, 50, "RIGHT", 2);
                        straightDriveEncoder(0.4, -10, 0.5);
                        //turnEncoder(0.5,25,"CC",0.75);
                        turnClamp("PERP", 250);
                        straightDriveEncoder(0.4, 20, 0.75);
                        strafeDriveEncoder(0.2, 20, "LEFT", 1);
                        clamp("CLOSE", 250);
                        strafeDriveEncoder(0.2, 35, "LEFT", 2);
                        turnEncoder(0.5, 166, "C", 1.75);
                        straightDriveEncoder(0.7, 260, 2);
                        clamp("OPEN", 250);
                        straightDriveEncoder(0.4, -40, 1);
                    }
                    break;
                case "MIDDLE":
                    if (!isStopRequested() && opModeIsActive()) {
                        straightDriveEncoder(0.2, -9, 0.75);
                        strafeDriveEncoder(1, 15, "RIGHT", 1.3);
                        actuator.setPower(1);//TODO use method created however it requires encoders on actuator
                        sleep(500);
                        actuator.setPower(0);
                        clamp("CLOSE", 250);
                        straightDriveEncoder(0.6, -30, 0.75);
                        turnEncoder(0.5, 86, "C", 1);
                        straightDriveEncoder(0.7, 160, 2.25);
                        clamp("OPEN", 250);
                        straightDriveEncoder(0.7, -235, 2.5);
                        turnEncoder(0.5, 87, "CC", 1);
                        clamp("OPEN", 250);
                        straightDriveEncoder(0.4, 40, 1.75);
                        clamp("CLOSE", 250);
                        straightDriveEncoder(0.6, -50, 0.75);
                        turnEncoder(0.5, 90, "C", 1.5);
                        straightDriveEncoder(0.7, 220, 2.25);
                        turnClamp("PERP", 250);
                        straightDriveEncoder(.7, 18, 1.5);
                        clamp("OPEN", 250);
                        straightDriveEncoder(.7, -40, 1.5);
                    }
                    break;
                case "BRIDGE":
                    if (!isStopRequested() && opModeIsActive()) {
                        straightDriveEncoder(0.2, -9, 0.75);
                        strafeDriveEncoder(0.4, 25, "RIGHT", .75);
                        actuator.setPower(1);//TODO use method created
                        sleep(500);
                        actuator.setPower(0);
                        clamp("CLOSE", 250);
                        straightDriveEncoder(0.2, -30, 1);
                        turnEncoder(.4, 84, "C", 1);
                        straightDriveEncoder(0.6, 130, 1.5);
                        clamp("OPEN", 250);
                        straightDriveEncoder(.5, -205, 3);
                        turnEncoder(0.5, 82, "CC", 1);
                        clamp("OPEN", 250);
                        straightDriveEncoder(0.3, 52, 2.25);
                        clamp("CLOSE", 250);
                        straightDriveEncoder(.5, -55, 1);
                        turnEncoder(0.5, 85, "C", 1);
                        straightDriveEncoder(0.7, 220, 2);
                        clamp("OPEN", 250);
                        straightDriveEncoder(0.4, -45, 1.75);
                    }
                    break;
            }
        }
    }
}
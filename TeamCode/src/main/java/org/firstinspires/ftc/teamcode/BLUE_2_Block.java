package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUE_2_BLOCK")
public class BLUE_2_Block extends Auto_Methods {


    @Override
    public void runOpMode() throws InterruptedException {
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
            rightFoundation.setPosition(.9);
            sleep(1000);
            straightDriveEncoder(0.2, 90, 2.5);// TODO adjust tomorrow morning
            skystoneColorScan("BLUE");
            telemetry.addLine("Skystone position: " + positionSkystone);
            telemetry.update();
            switch (positionSkystone) {
                case "WALL":
                    if (!isStopRequested() && opModeIsActive()) {
                        straightDriveEncoder(0.2, -9, 0.75);
                        strafeDriveEncoder(1, 10, "RIGHT", 0.75);
                        actuator.setPower(1);//TODO use method created however it requires encoders on actuator
                        sleep(400);
                        actuator.setPower(0);
                        clamp.setPosition(.25);
                        sleep(1300);
                        straightDriveEncoder(0.6, -30, 0.75);
                        turnEncoder(0.5, 88, "CC", 1);
                        straightDriveEncoder(0.7, 190, 1.75);
                        clamp("OPEN", 250);
                        straightDriveEncoder(0.7, -217, 2);
                        turnEncoder(0.5, 168, "CC", 1.75);
                        strafeDriveEncoder(0.5, 50, "LEFT", 2);
                        straightDriveEncoder(0.4, -10, 0.5);
                        //turnEncoder(0.5,25,"CC",0.75);
                        turnClamp("PERP", 250);
                        straightDriveEncoder(0.4, 20, 0.75);
                        strafeDriveEncoder(0.2, 20, "RIGHT", 1);
                        clamp("CLOSE", 250);
                        strafeDriveEncoder(0.2, 35, "RIGHT", 2);
                        turnEncoder(0.5, 166, "CC", 1.75);
                        straightDriveEncoder(0.7, 260, 2);
                        clamp("OPEN", 250);
                        straightDriveEncoder(0.4, -40, 1);
                    }
                    break;
                case "MIDDLE":
                    if (!isStopRequested() && opModeIsActive()) {
                        gyroDrive(.3,-4,0);
                        strafeDriveEncoder(1, 10, "LEFT", 1);
                        actuator.setPower(1);//TODO use method created however it requires encoders on actuator
                        sleep(400);
                        actuator.setPower(0);
                        clamp.setPosition(.25);
                        sleep(800);
                        gyroDrive(0.3, -25, 0);
                        turnEncoder(0.5, 76, "CC", 1);
                        straightDriveEncoder(0.7, 140, 2.25);
                        clamp("OPEN", 250);
                        straightDriveEncoder(0.7, -215, 2.5);
                        turnEncoder(0.5, 83, "C", 1);
                        clamp("OPEN", 250);
                        straightDriveEncoder(0.4, 27, 1.75);
                        clamp("CLOSE", 250);
                        straightDriveEncoder(0.6, -40, 0.75);
                        turnEncoder(0.5, 87, "CC", 1.5);
                        straightDriveEncoder(0.9, 220, 2.25);
//                        turnClamp("PERP", 250);

                        clamp("OPEN", 250);
                        straightDriveEncoder(.7, -50, 1.5);
                    }
                    break;
                case "BRIDGE":
                    if (!isStopRequested() && opModeIsActive()) {
                        straightDriveEncoder(0.2, -6, 0.75);
                        strafeDriveEncoder(0.4, 25, "LEFT", .75);
                        actuator.setPower(1);//TODO use method created however it requires encoders on actuator
                        sleep(400);
                        actuator.setPower(0);
                        clamp.setPosition(.25);
                        sleep(800);
                        gyroDrive(0.3, -30, 0);
                        turnEncoder(0.5, 76, "CC", 1);
                        straightDriveEncoder(0.6, 130, 1.5);
                        clamp("OPEN", 250);
                        straightDriveEncoder(.5, -205, 3);
                        turnEncoder(0.5, 82, "C", 1);
                        clamp("OPEN", 250);
                        straightDriveEncoder(0.3, 48, 2.25);
                        clamp("CLOSE", 250);
                        straightDriveEncoder(.5, -55, 1);
                        turnEncoder(0.5, 85, "CC", 1);
                        straightDriveEncoder(0.7, 220, 2);
                        clamp("OPEN", 250);
                        straightDriveEncoder(0.4, -45, 1.75);
                    }
                    break;
            }
        }
    }
}

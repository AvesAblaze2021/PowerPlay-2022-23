package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ablaze.autonomous.AblazeMove;
import org.firstinspires.ftc.teamcode.ablaze.autonomous.IMUDrive;
import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

@Autonomous
public class IMUTests extends LinearOpMode {

    private AblazeRobot ablazeRobot = new AblazeRobot();
    private AblazeMove ablazeMove = new AblazeMove();
    private IMUDrive imuDrive = new IMUDrive();
    Orientation lastAngles = new Orientation();
    ElapsedTime runtime = new ElapsedTime();
    private double wheelPower = 0.2;
    private int degrees = 90;
    private double timeoutMS = 3000;

    @Override
    public void runOpMode() {

        //Initialize the ablazeRobot and ablazeMove
        ablazeRobot.initialize(hardwareMap);
        ablazeMove.initialize(ablazeRobot, imuDrive, runtime);

        //Intialize IMU and IMU parameters
        imuDrive.initialize(ablazeRobot);
        imuDrive.initializeImu();

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        //Check if IMU is calibrated
        while (!isStopRequested() && !ablazeRobot.getImu().isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("IMU calib status", ablazeRobot.getImu().getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            runtime.reset();

            telemetry.addData("Correction: ", imuDrive.checkDirection());
            telemetry.update();


            //Test strafe with IMU
            ablazeMove.strafeLeftIMU(wheelPower, timeoutMS);

            //Test for moving forward with IMU
            ablazeMove.driveForwardIMU(wheelPower, timeoutMS);


            /*
            //Test for driving back with IMU
            ablazeMove.driveBackwardIMU(wheelPower, timeoutMS);
             */
            /*
            //Test for strafing right with IMU
            ablazeMove.strafeRightIMU(wheelPower);
            sleep(500);
            */

            /*
            //Test for strafing left with IMU
            ablazeMove.strafeLeftIMU(wheelPower);
            sleep(500);
            */

            //Test for turning right with IMU
            //imuDrive.rotate(degrees, wheelPower);
            break;
        }
    }
}
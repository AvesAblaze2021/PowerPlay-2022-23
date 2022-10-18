package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;


@Autonomous
public class AllMotorPowerTest extends LinearOpMode {
    private AblazeRobot ablazeRobot = new AblazeRobot();
    private double motorPower = ablazeRobot.getDefaultPower();

    @Override
    public void runOpMode(){
        // Initialize Robot
        ablazeRobot.initialize(hardwareMap);

        waitForStart();

        // Set Power
        ablazeRobot.driveTime(motorPower);
        while (opModeIsActive()) {
            sleep(50000);
        }
        ablazeRobot.stop();
    }
}
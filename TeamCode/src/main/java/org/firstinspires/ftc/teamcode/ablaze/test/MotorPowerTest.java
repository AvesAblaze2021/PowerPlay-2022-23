package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;


@Autonomous
public class MotorPowerTest extends LinearOpMode {
    private AblazeRobot ablazeRobot = new AblazeRobot();
    private DcMotor testMotor;
    private double motorPower = ablazeRobot.getDefaultPower();

    @Override
    public void runOpMode(){
        // Initialize Robot
        ablazeRobot.initialize(hardwareMap);

        waitForStart();

        //Change method here to test specific motor
        testMotor = ablazeRobot.getRightBackDrive();

        // Set Power
        testMotor.setPower(motorPower);
        while (opModeIsActive()) {
            sleep(50000);
        }
        testMotor.setPower(0);
    }
}
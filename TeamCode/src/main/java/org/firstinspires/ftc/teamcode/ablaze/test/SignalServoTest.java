package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

@Autonomous
public class SignalServoTest extends LinearOpMode {
    private AblazeRobot ablazeRobot;
    private double signalPos1 = 0.8;
    private double signalPos2 = 0.6;

    @Override
    public void runOpMode(){
        ablazeRobot = new AblazeRobot();
        ablazeRobot.initialize(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            ablazeRobot.getSignalServo().setPosition(signalPos1);
            ablazeRobot.getSignalServo().setPosition(signalPos2);
            break;
        }
    }
}
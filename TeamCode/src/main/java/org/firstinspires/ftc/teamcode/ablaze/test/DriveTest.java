package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FreightFrenzy202122.ablaze.common.PracticeRobot;
import org.firstinspires.ftc.teamcode.ablaze.common.*;

//Runs each motor once
@Autonomous
public class DriveTest extends LinearOpMode {
    PracticeRobot robot = new PracticeRobot();

    @Override
    public void runOpMode(){
        robot.initialize(hardwareMap);
        double power = robot.getDefaultPower();
        int time = 1000;
        
        //LF Motor
        robot.getLeftFrontDrive().setPower(power);
        sleep(time);

        //RF Motor
        robot.getRightFrontDrive().setPower(power);
        sleep(time);
        
        //RB Motor
        robot.getRightBackDrive().setPower(power);
        sleep(time);
        
        //LB Motor
        robot.getLeftBackDrive().setPower(power);
        sleep(time);
    }
}

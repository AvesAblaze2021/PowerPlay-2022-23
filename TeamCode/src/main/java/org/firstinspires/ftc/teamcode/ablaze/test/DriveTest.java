package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FreightFrenzy202122.ablaze.common.PracticeRobot;
import org.firstinspires.ftc.teamcode.ablaze.common.*;

//Runs each motor once
@Autonomous
public class DriveTest extends LinearOpMode {
    PracticeRobot robot;

    @Override
    public void runOpMode(){
        robot = new PracticeRobot();
        robot.initialize(hardwareMap);

        //LF Motor


        //RF Motor

        //RB Motor

        //LB Motor


    }
}

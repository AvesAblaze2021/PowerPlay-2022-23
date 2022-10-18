package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ablaze.common.PracticeRobot;

//Teleop test program for controlling the drivetrain
@TeleOp
public class TeleopDriveTest extends LinearOpMode {
    PracticeRobot robot = new PracticeRobot();

    @Override
    public void runOpMode(){
        robot.initialize(hardwareMap);
        double power = robot.getDefaultPower();
        
    }
}

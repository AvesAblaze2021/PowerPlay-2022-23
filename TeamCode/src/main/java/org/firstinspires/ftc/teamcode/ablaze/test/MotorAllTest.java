package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ablaze.autonomous.AblazeMove;
import org.firstinspires.ftc.teamcode.ablaze.autonomous.IMUDrive;
import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;


@Autonomous
public class MotorAllTest extends LinearOpMode {
    private AblazeRobot ablazeRobot = new AblazeRobot();
    private AblazeMove ablazeMove = new AblazeMove();
    private IMUDrive imuDrive = new IMUDrive();
    ElapsedTime runtime = new ElapsedTime();
    private double motorPower = .2;

    @Override
    public void runOpMode(){
        // Initialize Robot
        ablazeRobot.initialize(hardwareMap);
        ablazeMove.initialize(ablazeRobot, imuDrive, runtime);

        waitForStart();

        // Set Power
        ablazeMove.driveForward(motorPower);
        while (opModeIsActive()) {
            sleep(50000);
        }
        ablazeMove.driveStop();
    }
}
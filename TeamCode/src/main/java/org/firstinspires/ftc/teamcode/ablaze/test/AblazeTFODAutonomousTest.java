package org.firstinspires.ftc.teamcode.ablaze.test;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ablaze.autonomous.AblazeTFOD;
import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

public class AblazeTFODAutonomousTest extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private AblazeRobot ablazeRobot = new AblazeRobot();
    private AblazeTFOD tfod = new AblazeTFOD();
    private int milliseconds = 0;

    private double motorPower = .5;

    @Override
    public void runOpMode() throws InterruptedException {
        ablazeRobot.initialize(hardwareMap);

        tfod.initialize(hardwareMap, ablazeRobot);
        waitForStart();

        while(opModeIsActive()) {
            tfod.detectElement();

            milliseconds = tfod.time();
            telemetry.addData("Time: ", milliseconds);

            ablazeRobot.getLeftBackDrive().setPower(motorPower);
            ablazeRobot.getRightBackDrive().setPower(motorPower);
            ablazeRobot.getLeftFrontDrive().setPower(motorPower);
            ablazeRobot.getRightFrontDrive().setPower(motorPower);
            sleep(milliseconds);
        }
    }
}

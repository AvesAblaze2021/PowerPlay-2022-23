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
    private int zone;

    private double motorPower = .5;

    @Override
    public void runOpMode() throws InterruptedException {
        ablazeRobot.initialize(hardwareMap);

        tfod.initialize(hardwareMap, ablazeRobot);
        waitForStart();

        while(opModeIsActive()) {
            zone = tfod.detectElement();

            telemetry.addData("Zone #: ", zone);

            if(zone == 1) {
                ablazeRobot.getRightBackDrive().setPower(motorPower);
                ablazeRobot.getLeftBackDrive().setPower(motorPower);
                ablazeRobot.getLeftFrontDrive().setPower(motorPower);
                ablazeRobot.getRightFrontDrive().setPower(motorPower);
                sleep(500);
            } else if(zone == 2) {
                ablazeRobot.getRightBackDrive().setPower(motorPower);
                ablazeRobot.getLeftBackDrive().setPower(motorPower);
                ablazeRobot.getLeftFrontDrive().setPower(motorPower);
                ablazeRobot.getRightFrontDrive().setPower(motorPower);
                sleep(1000);
            } else if (zone == 3) {
                ablazeRobot.getRightBackDrive().setPower(motorPower);
                ablazeRobot.getLeftBackDrive().setPower(motorPower);
                ablazeRobot.getLeftFrontDrive().setPower(motorPower);
                ablazeRobot.getRightFrontDrive().setPower(motorPower);
                sleep(1500);
            } if(zone == 1) {
                ablazeRobot.getRightBackDrive().setPower(motorPower);
                ablazeRobot.getLeftBackDrive().setPower(motorPower);
                ablazeRobot.getLeftFrontDrive().setPower(motorPower);
                ablazeRobot.getRightFrontDrive().setPower(motorPower);
                sleep(2000);
            }

        }
    }
}

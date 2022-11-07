package org.firstinspires.ftc.teamcode.ablaze.common;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

@Autonomous
public class RRNavigation implements Navigation {
    AblazeRobot ablazeRobot;
    public RRNavigation(AblazeRobot ablazeRobot){
        this.ablazeRobot = ablazeRobot;
    }

    private void driveit(double frontLeftMotor, double backLeftMotor,
                         double frontRightMotor, double backRightMotor, long sleepTime) {
        ablazeRobot.getLeftBackDrive().setPower(backLeftMotor);
        ablazeRobot.getLeftFrontDrive().setPower(frontLeftMotor);
        ablazeRobot.getRightFrontDrive().setPower(frontRightMotor);
        ablazeRobot.getRightBackDrive().setPower(backRightMotor);
        Thread.sleep(sleepTime);
        ablazeRobot.frontLeftMotor.setPower(0.0);
        ablazeRobot.frontRightMotor.setPower(0.0);
        ablazeRobot.backLeftMotor.setPower(0.0);
        ablazeRobot.backRightMotor.setPower(0.0);
    }

}

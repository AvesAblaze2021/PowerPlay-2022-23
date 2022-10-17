package org.firstinspires.ftc.teamcode.ablaze.test;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

@Autonomous
public class ClawTest extends LinearOpMode {
    private AblazeRobot ablazeRobot = new AblazeRobot();

    @Override
    public void runOpMode(){
        ablazeRobot.initialize(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            ablazeRobot.getArmClaw().setPosition(ablazeRobot.getClawMaxPos());

            sleep(5000);

            ablazeRobot.getArmClaw().setPosition(ablazeRobot.getClawMinPos());

            sleep(5000);

            break;
        }
    }
}

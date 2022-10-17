package org.firstinspires.ftc.teamcode.ablaze.test;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

@Autonomous
public class LightTest extends LinearOpMode {
    private AblazeRobot ablazeRobot = new AblazeRobot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        ablazeRobot.initialize(hardwareMap);

        waitForStart();

        //LED light = ablazeRobot.getLight();
        runtime.reset();
        while(runtime.milliseconds() < 10000){
            //light.enable(true);
            sleep(1000);
            //light.enable(false);
            sleep(1000);
        }
    }

}

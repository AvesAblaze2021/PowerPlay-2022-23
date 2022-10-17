package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;


@Autonomous
public class ArmTicks extends LinearOpMode {
    private AblazeRobot ablazeRobot = new AblazeRobot();
    private DcMotor armMotor;
    private double armTicks;

    @Override
    public void runOpMode(){

        // Initialize Robot
        ablazeRobot.initialize(hardwareMap);

        // Init armMotor
        armMotor = ablazeRobot.getArmMotor();

        // Enable Encoder
        ablazeRobot.useEncoderForArm(true);

        waitForStart();
        
        while (opModeIsActive()) {
            armTicks = armMotor.getMotorType().getTicksPerRev();
            telemetry.addData("Arm Ticks: ", armTicks);
            telemetry.update();
        } // while end
    } // runOpMode end
} // class end

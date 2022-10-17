package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

@Autonomous
public class FreightDeliveryTest extends LinearOpMode{
    private AblazeRobot ablazeRobot = new AblazeRobot();
  
    //Autonomous implementation doesn't need CAP, Teleop does
    private enum Level {TOP, MIDDLE, BOTTOM, CAP};
  
    //Change for each Level
    private Level targetLevel = Level.TOP;
  
    //For Autonomous implementation, place first 3 values in AblazeMove
    //For Teleop implementation, place last value in AblazeDrive
    private double topLevelPos = 1.0; //final
    private double midLevelPos = 0.82; //final
    private double bottomLevelPos = 0.8; //final
    private double capLevelPos = 0.9;
  
    @Override
    public void runOpMode() throws InterruptedException{
        ablazeRobot.initialize(hardwareMap);
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            //Move arm completely down - tweak encoder values
            moveArm(0.148, 1, 300);

            //Move arm down to target Level
            if (targetLevel == Level.TOP) {

                //Open claw
                //ablazeRobot.getArmClaw().setPosition(ablazeRobot.getClawMaxPos());

                //Close claw
                //ablazeRobot.getArmClaw().setPosition(ablazeRobot.getClawMinPos());

                //Reset Arm Control Servo Position
                //ablazeRobot.getArmControl().setPosition(0);

            }

            if (targetLevel == Level.MIDDLE) {

                //Move Arm Control Servo to hold Arm
                //ablazeRobot.getArmControl().setPosition(midLevelPos);

                //Open claw
                //ablazeRobot.getArmClaw().setPosition(ablazeRobot.getClawMaxPos());

                //Close claw
                //ablazeRobot.getArmClaw().setPosition(ablazeRobot.getClawMinPos());

                //Reset Arm Control Servo Position
                //ablazeRobot.getArmControl().setPosition(0);

            }

            if (targetLevel == Level.BOTTOM) {

                //Move Arm Control Servo to hold Arm
                //ablazeRobot.getArmControl().setPosition(bottomLevelPos);

                //Open claw
                //ablazeRobot.getArmClaw().setPosition(ablazeRobot.getClawMaxPos());

                //Close claw
                //ablazeRobot.getArmClaw().setPosition(ablazeRobot.getClawMinPos());

                //Reset Arm Control Servo Position
                //ablazeRobot.getArmControl().setPosition(0);

            }

            if (targetLevel == Level.CAP) {

                //Move Arm Control Servo to hold Arm
                //ablazeRobot.getArmControl().setPosition(capLevelPos);

                //Open claw
                //ablazeRobot.getArmClaw().setPosition(ablazeRobot.getClawMaxPos());

                //Close claw
                //ablazeRobot.getArmClaw().setPosition(ablazeRobot.getClawMinPos());

                //Reset Arm Control Servo Position
                //ablazeRobot.getArmControl().setPosition(0);

            }
            break;
        }
    }

    public void moveArm(double speed, int direction, int ticks) {
        int pos = ablazeRobot.getArmMotor().getCurrentPosition() + ticks;
        if (direction == -1) {
            ablazeRobot.getArmMotor().setTargetPosition(-pos);
        }
        if (direction == 1) {
            ablazeRobot.getArmMotor().setTargetPosition(pos);
        }
        ablazeRobot.getArmMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ablazeRobot.getArmMotor().setPower(speed);
        sleep(1200);
        ablazeRobot.getArmMotor().setPower(0);
    }
}
package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

@Autonomous
public class ArmControlTest extends LinearOpMode{
    private AblazeRobot ablazeRobot = new AblazeRobot();
    private double armControlPower = 0.5;
    private double armMotorPower = 0.5;

    //Level 3: ticks: 450, timeout: 600
    //private int armControlTicks = 450;
    //private int timeoutMS = 600;

    //Level 2: ticks: 950, timeout 1200
    //private int armControlTicks = 950;
    //private int timeoutMS = 1200;

    //Level 1: 1350, timeout 1800
    private int armControlTicks = 1350;
    private int timeoutMS = 1800;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status: ", "Initializing...");
        telemetry.update();

        ablazeRobot.initialize(hardwareMap);

        telemetry.addData("Status: ", "Initialized!");
        telemetry.update();

        waitForStart();

        moveArmDown(2100, 2000);

        sleep(1000);

        moveArmUp(armControlTicks, timeoutMS);

        sleep(1000);
    }

    public void moveArmDown(int moveTicks,
                            double timeoutMS)  {
        // Adjust Zero Power Behaviour
        ablazeRobot.getArmMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ablazeRobot.getArmControl().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        ablazeRobot.getArmControl().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ablazeRobot.getArmControl().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ablazeRobot.getArmMotor().setTargetPosition(moveTicks);
        ablazeRobot.getArmMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ablazeRobot.getArmMotor().setPower(armMotorPower);
        runtime.reset();
        while (opModeIsActive() &&
                (runtime.milliseconds() < timeoutMS) &&
                (ablazeRobot.getArmMotor().isBusy())) {
            telemetry.addData("Arm Motor Position: ", ablazeRobot.getArmMotor().getCurrentPosition());
            telemetry.update();
        }
        ablazeRobot.getArmMotor().setPower(0);
        ablazeRobot.useEncoderForArm(true);
    }

    public void moveArmUp(int moveTicks,
                          double timeoutMS)  {
        // Adjust Zero Power Behaviour
        ablazeRobot.getArmMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ablazeRobot.getArmControl().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ablazeRobot.getArmControl().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ablazeRobot.getArmControl().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ablazeRobot.getArmControl().setTargetPosition(moveTicks);
        ablazeRobot.getArmControl().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ablazeRobot.getArmControl().setPower(armControlPower);
        runtime.reset();
        while (opModeIsActive() &&
                (runtime.milliseconds() < timeoutMS) &&
                (ablazeRobot.getArmControl().isBusy())) {
            telemetry.addData("Arm Control Position: ", ablazeRobot.getArmControl().getCurrentPosition());
            telemetry.update();
        }
        ablazeRobot.getArmControl().setPower(0);
        ablazeRobot.useEncoderForArm(true);
        sleep(1000);
    }
}

package org.firstinspires.ftc.teamcode.ablaze.test;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;
@Autonomous
public class EncoderNavTest extends LinearOpMode {
    private AblazeRobot robot = new AblazeRobot();
    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor brMotor;
    private DcMotor blMotor;
    private double motorPower = 0.3;

    @Override
    public void runOpMode() {
        robot.initialize(hardwareMap);
        flMotor = robot.getLeftFrontDrive();
        frMotor = robot.getRightFrontDrive();
        brMotor = robot.getRightBackDrive();
        blMotor = robot.getLeftBackDrive();

        waitForStart();

        encoderDrive(500, "forward");

    }

    public void encoderDrive(int position, String direction){
        flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(direction == "left"){
            flMotor.setTargetPosition(-position);
            blMotor.setTargetPosition(-position);
            brMotor.setTargetPosition(position);
            frMotor.setTargetPosition(position);
        } else if(direction == "right"){
            flMotor.setTargetPosition(position);
            blMotor.setTargetPosition(position);
            brMotor.setTargetPosition(-position);
            frMotor.setTargetPosition(-position);
        } else{
            flMotor.setTargetPosition(position);
            blMotor.setTargetPosition(position);
            brMotor.setTargetPosition(position);
            frMotor.setTargetPosition(position);
        }

        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flMotor.setPower(motorPower);
        blMotor.setPower(motorPower);
        brMotor.setPower(motorPower);
        frMotor.setPower(motorPower);

        while (flMotor.isBusy() && blMotor.isBusy() && brMotor.isBusy() && frMotor.isBusy()) {
            continue;
        }

        flMotor.setPower(0.0);
        blMotor.setPower(0.0);
        brMotor.setPower(0.0);
        frMotor.setPower(0.0);
    }
}

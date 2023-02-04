package org.firstinspires.ftc.teamcode.ablaze.common;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
public class Navigation {
    private SampleMecanumDrive drive;
    private AblazeRobot robot;
    private DcMotor flMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;
    private DcMotor frMotor;
    private double motorPower;

    public void moveForward(int inches){
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(inches).build());
        drive.updatePoseEstimate();
    }

    public void moveBackward(int inches){
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(-inches).build());
        drive.updatePoseEstimate();
    }

    public void turn(int degrees){
        drive.turn(Math.toRadians(degrees));
        drive.updatePoseEstimate();
    }

    public void initialize(HardwareMap hardwareMap, AblazeRobot robot){
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        this.robot = robot;
        flMotor = robot.getLeftFrontDrive();
        frMotor = robot.getRightFrontDrive();
        blMotor = robot.getLeftBackDrive();
        brMotor = robot.getRightBackDrive();
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

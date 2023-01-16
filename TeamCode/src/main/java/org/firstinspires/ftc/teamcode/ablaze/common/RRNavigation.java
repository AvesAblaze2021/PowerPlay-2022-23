package org.firstinspires.ftc.teamcode.ablaze.common;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous()
public class RRNavigation {
    SampleMecanumDrive drive;
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

    public void initialize(HardwareMap hardwareMap){
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
    }
}

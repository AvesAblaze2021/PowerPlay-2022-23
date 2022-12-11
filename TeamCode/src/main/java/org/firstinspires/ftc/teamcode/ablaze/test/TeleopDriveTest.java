package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ablaze.common.PracticeRobot;

//Teleop test program for controlling the drivetrain
@TeleOp
public class TeleopDriveTest extends OpMode {
    PracticeRobot robot = new PracticeRobot();
    boolean isLoop = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //Init hardware
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Initialize Robot
        robot.initialize(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     *
     */
    @Override
    public void loop(){
        isLoop = true;
        driveWithTwoJoysticks();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        isLoop = false;
    }

    // drive with joysticks
    public void driveWithTwoJoysticks() {
        double drivingPower = gamepad1.left_stick_y;
        double turningPower = gamepad1.right_stick_x;
        double strafing = gamepad1.left_stick_x;
        if (Math.abs(strafing) < 0.03) strafing = 0;
        double leftBackPower = drivingPower - turningPower + strafing;
        double leftFrontPower = drivingPower - turningPower - strafing;
        double rightFrontPower = drivingPower + turningPower + strafing;
        double rightBackPower = drivingPower + turningPower - strafing;
        if(gamepad1.left_bumper) {
            leftBackPower /= 3;
            leftFrontPower /= 3;
            rightFrontPower /= 3;
            rightBackPower /= 3;
        }
        else if(gamepad1.right_bumper) {
            leftBackPower /= 1.2;
            leftFrontPower /= 1.2;
            rightFrontPower /= 1.2;
            rightBackPower /= 1.2;
        }
        else if (isLoop){
            leftBackPower /= 1.6;
            leftFrontPower /= 1.6;
            rightFrontPower /= 1.6;
            rightBackPower /= 1.6;
        }
        else{
            leftBackPower /= 1.6;
            leftFrontPower /= 1.6;
            rightFrontPower /= 1.6;
            rightBackPower /= 1.6;
        }

        //sets the speed of the drive motors
        robot.getLeftBackDrive().setPower(leftBackPower);
        robot.getLeftFrontDrive().setPower(leftFrontPower);
        robot.getRightFrontDrive().setPower(rightFrontPower);
        robot.getRightBackDrive().setPower(rightBackPower);
    }
}

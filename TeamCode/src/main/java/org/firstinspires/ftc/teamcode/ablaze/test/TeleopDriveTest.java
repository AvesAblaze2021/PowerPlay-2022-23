package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ablaze.common.PracticeRobot;

//Teleop test program for controlling the drivetrain
@TeleOp
public class TeleopDriveTest extends LinearOpMode {
    PracticeRobot robot = new PracticeRobot();

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
        driveWithTwoJoysticks();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stop();
    }

    // drive with joysticks
    public void driveWithTwoJoysticks() {
        double px = gamepad1.left_stick_x;
        double py = -gamepad1.left_stick_y;
        double pa = (gamepad1.right_stick_x - gamepad1.right_stick_y);
        if (Math.abs(pa) < 0.03) pa = 0;
        double p1 = -px + py + pa;
        double p2 = px + py + pa;
        double p3 = -px + py - pa;
        double p4 = px + py - pa;
        if(gamepad1.left_bumper) {
            p1 /= 3;
            p2 /= 3;
            p3 /= 3;
            p4 /= 3;
        }
        if(gamepad1.right_bumper) {
            p1 /= 1.2;
            p2 /= 1.2;
            p3 /= 1.2;
            p4 /= 1.2;
        }
        else if (isLoop){
            p1 /= 1.6;
            p2 /= 1.6;
            p3 /= 1.6;
            p4 /= 1.6;
        }

        //sets the speed of the drive motors
        ablazeRobot.getLeftBackDrive().setPower(p1);
        ablazeRobot.getLeftFrontDrive().setPower(p2);
        ablazeRobot.getRightFrontDrive().setPower(p3);
        ablazeRobot.getRightBackDrive().setPower(p4);
    }
}

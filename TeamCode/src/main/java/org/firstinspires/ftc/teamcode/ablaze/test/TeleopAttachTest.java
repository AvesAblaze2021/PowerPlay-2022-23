//OFFICIAL TELEOP
package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

@TeleOp (name = "TeleOpAttachTest")
public class TeleopAttachTest extends OpMode {
    private AblazeRobot robot = new AblazeRobot();
    private boolean isLoop = false;
    private DcMotor verticalSlideMotor;
    private Servo clawServo;
    private double motorPower;
    private double clawOpenPos = 0.06;
    private double clawClosePos = 0.30;
    private int lastPos = 0;
    private enum AttachmentState{
        START, //Initial State - gamepad monitoring occurs here
        UP,
        DOWN,
        CLOSE, //Close claws
        OPEN //Open claws
    };
    private ElapsedTime runtime = new ElapsedTime();
    private AttachmentState attachState = AttachmentState.START;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //Notify init start
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        //Set default motor power
        motorPower = robot.getDefaultPower();

        //Initialize Hardware
        robot.initialize(hardwareMap);
        verticalSlideMotor = robot.getVerticalSlideMotor();
        clawServo = robot.getClawServo();
        runtime.reset();

        //Teleop's init state is set at the end of Autonomous

        //Notify init end
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
        isLoop = true;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     *
     */
    @Override
    public void loop(){
        //Check all states
        switch(attachState){
            case START: //Initial state - all gamepad conditionals go here, leads to other States
                if(gamepad2.y) {
                    attachState = AttachmentState.UP;
                }
                if(gamepad2.left_bumper){
                    attachState = AttachmentState.OPEN;
                }
                if(gamepad2.right_bumper){
                    attachState = AttachmentState.CLOSE;
                }
                break;

            case UP:
                moveLinearSlides(100);
                attachState = AttachmentState.START;
                break;

            case DOWN:
                moveLinearSlides(-100);
                attachState = AttachmentState.START;
                break;

            case OPEN: //State for opening claw - delivery
                clawServo.setPosition(clawOpenPos);
                attachState = AttachmentState.START;
                break;

            case CLOSE: //State for closing claw - pickup
                clawServo.setPosition(clawClosePos);
                attachState = AttachmentState.START;
                break;
        }
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
        telemetry.addData("gamepad1", "left stick moved", ".");
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

    //Moves slides to tick pos
    public void moveLinearSlides(int ticks) {
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlideMotor.setTargetPosition(ticks);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor.setPower(motorPower);
        while (verticalSlideMotor.isBusy() && verticalSlideMotor.getCurrentPosition() < 700) {
            telemetry.addData("LFT, RFT", "Running to %7d", ticks);
            telemetry.addData("LFP, RFP", "Running at %7d",
                    verticalSlideMotor.getCurrentPosition()
            );
            telemetry.update();
        }
        verticalSlideMotor.setPower(0);
        verticalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}

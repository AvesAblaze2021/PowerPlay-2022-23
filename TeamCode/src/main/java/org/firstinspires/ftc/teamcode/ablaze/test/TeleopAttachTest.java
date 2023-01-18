package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;
import org.firstinspires.ftc.teamcode.ablaze.teleop.AblazeTeleop;

//Teleop test program for testing attachments
@TeleOp
public class TeleopAttachTest extends OpMode {
    AblazeRobot robot = new AblazeRobot();

    private boolean isLoop = false;
    private final int[] vertical_level_ticks = {100, 1200, 2300, 3800}; //Tick values for every level
    private DcMotor verticalSlideMotor;
    private Servo rotationServo;
    private Servo clawServo;
    private final double motorPower = 0.3;
    private int vertical_level = 0; //Default position of arm - set at the end of auto
    private double clawOpenPos = 0.0;
    private double clawClosePos = 1.0;
    private double rotDeliverPos = 0.85;
    private double rotPickupPos = 0.21;
    private enum AttachmentState{
        START, //Initial State - gamepad monitoring occurs here
        UP, //Moves linear slide up one level
        DOWN, //Moves linear slide down one level
        LEFT, //Rotates arm left
        RIGHT, //Rotates arm right
        CLOSE, //Close claws
        OPEN, //Open claws
        AUTO_PICKUP, //Automates pickup process
        AUTO_DELIVER //Automates delivery process
    };

    private AttachmentState attachState = AttachmentState.START;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //Notify init start
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        //Initialize Hardware
        robot.initialize(hardwareMap);
        verticalSlideMotor = robot.getVerticalSlideMotor();
        rotationServo = robot.getRotationServo();
        clawServo = robot.getClawServo();

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
                if(gamepad2.y){
                    attachState = AttachmentState.UP;
                }
                if(gamepad2.a){
                    attachState = AttachmentState.DOWN;
                }
                if(gamepad2.x){
                    attachState = AttachmentState.LEFT;
                }
                if(gamepad2.b){
                    attachState = AttachmentState.RIGHT;
                }
                if(gamepad2.left_bumper){
                    attachState = AttachmentState.OPEN;
                }
                if(gamepad2.right_bumper){
                    attachState = AttachmentState.CLOSE;
                }
                if(gamepad2.dpad_up){
                    attachState = AttachmentState.AUTO_PICKUP;
                }
                if(gamepad2.dpad_down){
                    attachState = AttachmentState.AUTO_DELIVER;
                }
                break;

            case UP: //State for moving arm up one level
                if(vertical_level < vertical_level_ticks.length - 1){
                    vertical_level += 1;
                    moveLinearSlides();
                }
                attachState = AttachmentState.START;
                break;

            case DOWN: //State for moving arm down one level
                if(vertical_level > 0){
                    vertical_level -= 1;
                    moveLinearSlides();
                }
                attachState = AttachmentState.START;
                break;

            case LEFT: //State for moving arm left - pickup
                rotationServo.setPosition(rotPickupPos);
                attachState = AttachmentState.START;
                break;

            case RIGHT: //State for moving arm right - delivery
                rotationServo.setPosition(rotDeliverPos);
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
            
            case AUTO_PICKUP: //Automated state for pickup
                //Enforce initial positions
                initPickup();

                clawServo.setPosition(clawClosePos);
                vertical_level = 1;
                moveLinearSlides();
                rotationServo.setPosition(rotDeliverPos);

                attachState = AttachmentState.START;
                break;

            case AUTO_DELIVER: //Automated state for delivery
                //Enforce initial positions
                initDelivery();

                clawServo.setPosition(clawOpenPos);
                vertical_level = 0;
                moveLinearSlides();
                rotationServo.setPosition(rotPickupPos); //may run into junction - test further

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

    //Moves slides to currently set vertical level
    public void moveLinearSlides(){
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setTargetPosition(level_ticks[vertical_level]);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(motorPower);
        while (isLoop && slideMotor.isBusy()) {
            telemetry.addData("LFT, RFT", "Running to %7d", level_ticks[level]);
            telemetry.addData("LFP, RFP", "Running at %7d",
                    slideMotor.getCurrentPosition()
            );
            telemetry.addData("level", level);
            telemetry.update();
        }
        sleep(500);
    }

    //Moves robot to delivery state
    public void initDelivery(){
        rotationServo.setPosition(rotDeliverPos);
        clawServo.setPosition(clawClosePos);;
        vertical_level = 0;
        moveLinearSlides();
    }
    
    //Moves robot to pickup state
    public void initPickup(){
        rotationServo.setPosition(rotPickupPos);
        clawServo.setPosition(clawOpenPos);
        vertical_level = 0;
        moveLinearSlides();
    }
}

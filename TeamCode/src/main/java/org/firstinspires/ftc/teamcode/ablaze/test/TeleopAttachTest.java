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
    private final int[] vertical_level_ticks = {200, 1200, 2300, 3500}; //Tick values for every level
    private DcMotor verticalSlideMotor;
    private Servo rotationServo;
    private Servo scissorServo;
    private Servo alignServo;
    private final double motorPower = 0.3;
    private int vertical_level = 1; //Default position of arm - 0 is for picking up cones
    private double clawPickupPos = 1.0;
    private double clawDropPos = 0.0;
    private double rotDeliverPos = 0.85;
    private double rotPickupPos = 0.21;
    private enum AttachmentState{
        START, //Initial State - gamepad monitoring occurs here
        UP, //Moves linear slide up one level
        DOWN, //Moves linear slide down one level
        LEFT, //Rotates arm left
        RIGHT, //Rotates arm right
        PICKUP, //AUTOMATED: Picks up cone completely
        DROP //Move scissors right (from servo perspective)
    };

    private AttachmentState attachState = AttachmentState.START;

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
        verticalSlideMotor = robot.getVerticalSlideMotor();
        rotationServo = robot.getRotationServo();
        scissorServo = robot.getScissorServo();
        alignServo = robot.getAlignServo();

        moveLinearSlides(verticalSlideMotor, vertical_level_ticks, vertical_level);

        rotationServo.setPosition(0.21);
        scissorServo.setPosition(0.4);

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
                if(gamepad2.right_bumper){
                    attachState = AttachmentState.PICKUP;
                }
                if(gamepad2.left_bumper){
                    attachState = AttachmentState.DROP;
                }
                break;

            case UP: //State for moving arm up one level
                if(vertical_level < vertical_level_ticks.length - 1){
                    vertical_level += 1;
                    moveLinearSlides(verticalSlideMotor, vertical_level_ticks, vertical_level);
                }
                attachState = AttachmentState.START;
                break;

            case DOWN: //State for moving arm down one level
                if(vertical_level > 0){
                    vertical_level -= 1;
                    moveLinearSlides(verticalSlideMotor, vertical_level_ticks, vertical_level);
                }
                attachState = AttachmentState.START;
                break;

            case LEFT:
                rotationServo.setPosition(rotPickupPos);
                attachState = AttachmentState.START;
                break;

            case RIGHT:
                rotationServo.setPosition(rotDeliverPos);
                attachState = AttachmentState.START;
                break;

            case PICKUP:
                scissorServo.setPosition(clawPickupPos);
                attachState = AttachmentState.START;
                break;

            case DROP:
                scissorServo.setPosition(clawDropPos);
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

    // drive with joysticks - WIP
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

    public void moveLinearSlides(DcMotor slideMotor, int[] level_ticks, int level){
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setTargetPosition(level_ticks[level]);
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
    }
}

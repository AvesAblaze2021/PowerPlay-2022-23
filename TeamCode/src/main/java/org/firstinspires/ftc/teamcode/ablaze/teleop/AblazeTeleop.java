//Aves Ablaze 2021-22 Official States Teleop
package org.firstinspires.ftc.teamcode.ablaze.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;
import org.firstinspires.ftc.teamcode.ablaze.common.PracticeRobot;
@TeleOp(name="AblazeTeleop", group="AblazeRobot")
public class AblazeTeleop extends OpMode{
    AblazeRobot ablazeRobot = new AblazeRobot();
    ElapsedTime runtime = new ElapsedTime();

    private enum Signals {ARM_UP, ARM_DOWN, CLAW_OPEN, CLAW_CLOSE};
    private double signalUpPos = 0.1;
    private double signalDownPos = 0.3;
    private double signalClosePos = 0.5;
    private double signalOpenPos = 0.7;

    private double armControlPower = 0.7;
    private double armMotorPower = 0.5;
    private int level1PosFirst = 1450;
    private int level1PosAll = 1650;
    private int armDownTicks = 750;
    private int armTimes = 0;
    private boolean isLoop = false;

    //All possible States for all Attachments
    private enum AttachmentState{
        START, //Initial State - gamepad monitoring occurs here
        CHECK, //Arm
        OPEN, //Arm Claw
        CLOSE, //Arm Claw
        ACTIVATE, //Carousel
        DEACTIVATE //Carousel
    };

    private AttachmentState attachState = AttachmentState.START;
    private double requiredTimeoutMS = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Initialize Robot
        ablazeRobot.initialize(hardwareMap);

        // Use Encoder for Arm
        ablazeRobot.useEncoderForArm(true);

        // Do not Use Encoder for Wheels
        ablazeRobot.useEncoderForWheels(false);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop(){
        isLoop = true;
        //Check all states
        switch(attachState){
            case START: //Initial state - all gamepad conditionals go here, leads to other States
                arm();
                if(gamepad2.a){
                    attachState = AttachmentState.OPEN;
                }
                if(gamepad2.b){
                    attachState = AttachmentState.CLOSE;
                }
                if(gamepad2.x){
                    attachState = AttachmentState.ACTIVATE;
                }
                if(gamepad2.y){
                    attachState = AttachmentState.DEACTIVATE;
                }
                break;

            case CHECK: //Check if Arm has either reached target or if timeout has been reached
                        //Similar to while loop used in autonomous
                if((!ablazeRobot.getArmControl().isBusy() &&
                        !ablazeRobot.getArmMotor().isBusy()) ||
                        runtime.milliseconds() >= requiredTimeoutMS){
                    resetArmMotors();
                    attachState = AttachmentState.START;
                }
                break;

            case OPEN:
                ablazeRobot.getArmClaw().setPosition(ablazeRobot.getClawMaxPos());
                signal(Signals.CLAW_OPEN);
                attachState = AttachmentState.START;
                break;

            case CLOSE:
                ablazeRobot.getArmClaw().setPosition(ablazeRobot.getClawMinPos());
                signal(Signals.CLAW_CLOSE);
                attachState = AttachmentState.START;
                break;

            case ACTIVATE:
                ablazeRobot.getCarouselMotor().setPower(-0.6);
                attachState = AttachmentState.START;
                break;

            case DEACTIVATE:
                ablazeRobot.getCarouselMotor().setPower(0);
                attachState = AttachmentState.START;
                break;
        }

        //State machine doesn't interfere with drivetrain code here
        driveWithTwoJoysticks();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        isLoop = false;
        ablazeRobot.stop();
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

    //Arm controls
    public void arm(){
        //Regular Arm Up Control
        if(gamepad2.dpad_up){
            if(armTimes == 0){
                moveArmUp(level1PosFirst);
                requiredTimeoutMS = 1800;
            }
            else{
                moveArmUp(level1PosAll);
                requiredTimeoutMS = 2100;
            }
            signal(Signals.ARM_UP);
            armTimes++;
        }

        //Arm Down Control
        if(gamepad2.dpad_down){
            moveArmDown(armDownTicks);
            signal(Signals.ARM_DOWN);
            requiredTimeoutMS = 2000;
        }

        //Rewind arm control
        if(gamepad2.left_bumper){
            rewind();
            requiredTimeoutMS = 1000;
        }

        //Arm Up Adjust Control
        if(gamepad2.dpad_left){
            moveArmUp(50);
            signal(Signals.ARM_UP);
            requiredTimeoutMS = 250;
        }
/*
        //Initial Arm Up Control
        if(gamepad2.right_bumper){
            moveArmUp(level1PosFirst);
            signal(Signals.ARM_UP);
            requiredTimeoutMS = 750;
        }
 */

        //Arm Down Adjust Control
        if(gamepad2.dpad_right){
            moveArmDown(100);
            signal(Signals.ARM_DOWN);
            requiredTimeoutMS = 250;
        }

        attachState = AttachmentState.CHECK;
        runtime.reset();
    }

    //Move Arm Up
    private void moveArmUp(int ticks) {
        // Set Zero Power Behaviour
        ablazeRobot.getArmMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ablazeRobot.getArmControl().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run Using Encoder
        ablazeRobot.getArmControl().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Stop and rest Encoders
        ablazeRobot.getArmControl().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set New Target Position
        ablazeRobot.getArmControl().setTargetPosition(ticks);

        // Turn On RUN_TO_POSITION
        ablazeRobot.getArmControl().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Speed
        ablazeRobot.getArmControl().setPower(armControlPower);
    }

    //Rewind if needed
    private void rewind(){
        // Set Zero Power Behaviour
        ablazeRobot.getArmMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ablazeRobot.getArmControl().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run Using Encoder
        ablazeRobot.getArmControl().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Stop and rest Encoders
        ablazeRobot.getArmControl().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set New Target Position
        ablazeRobot.getArmControl().setTargetPosition(500);

        // Turn On RUN_TO_POSITION
        ablazeRobot.getArmControl().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Speed
        ablazeRobot.getArmControl().setPower(armControlPower/2);
    }

    //Move Arm Down - Regular
    private void moveArmDown(int ticks){
        // Adjust Zero Power Behaviour
        ablazeRobot.getArmMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ablazeRobot.getArmControl().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Run Using Encoder
        ablazeRobot.getArmMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Stop and rest Encoders
        ablazeRobot.getArmMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set New Target Position
        ablazeRobot.getArmMotor().setTargetPosition(ticks);

        // Turn On RUN_TO_POSITION
        ablazeRobot.getArmMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Speed
        ablazeRobot.getArmMotor().setPower(armMotorPower/2);
    }
    
    public void resetArmMotors(){
        ablazeRobot.getArmMotor().setPower(0);
        ablazeRobot.getArmControl().setPower(0);
        ablazeRobot.useEncoderForArm(true);
    }

    private void signal(Signals signal){
        switch(signal){
            case ARM_UP:
                ablazeRobot.getSignalServo().setPosition(signalUpPos);
                break;
            case ARM_DOWN:
                ablazeRobot.getSignalServo().setPosition(signalDownPos);
                break;
            case CLAW_OPEN:
                ablazeRobot.getSignalServo().setPosition(signalOpenPos);
                break;
            case CLAW_CLOSE:
                ablazeRobot.getSignalServo().setPosition(signalClosePos);
                break;
        }
    }
}

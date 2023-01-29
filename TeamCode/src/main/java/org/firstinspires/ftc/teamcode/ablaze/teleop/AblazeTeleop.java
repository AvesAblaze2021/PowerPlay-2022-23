//DO NOT USE THIS PROGRAM
//DO NOTTHIS PROGRAM
//DO NOT
//DO NOT
package org.firstinspires.ftc.teamcode.ablaze.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

@TeleOp(name = "AblazeTeleop")
public class AblazeTeleop extends OpMode {
    AblazeRobot robot = new AblazeRobot();
    ElapsedTime runtime = new ElapsedTime();

    private boolean isLoop = false;
    private int level = 0; //0 = Start, 1 = Low, 2 = Medium, 3 = High
    private final int[] level_ticks = {0, 500, 1000, 1500, 1700}; //Tick values for every level
    private DcMotor verticalSlideMotor;
    private final double motorPower = 0.3;

    //All possible States for all Attachments - change for Power Play
    private enum AttachmentState {
        START, //Initial State - gamepad monitoring occurs here
        UP, //Moves linear slide up one level
        DOWN, //Moves linear slide down one level
        LEFT, //Rotates arm left
        RIGHT //Rotates arm right
    }

    ;

    private AttachmentState attachState = AttachmentState.START;

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
        robot.initialize(hardwareMap);

        //Stop and reset vertical slide encoder
        verticalSlideMotor = robot.getVerticalSlideMotor();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        isLoop = true;
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
     * Change for Power Play
     */
    @Override
    public void loop() {
        isLoop = true;
        //Check all states
        switch (attachState) {
            case START: //Initial state - all gamepad conditionals go here, leads to other States
                if (gamepad2.y) {
                    attachState = AttachmentState.UP;
                }
                if (gamepad2.a) {
                    attachState = AttachmentState.DOWN;
                }
                if (gamepad2.x) {
                    telemetry.addData("button", "x");
                    telemetry.update();
                    attachState = AttachmentState.LEFT;
                }
                if (gamepad2.b) {
                    attachState = AttachmentState.RIGHT;
                }
                break;

            case UP: //State for moving arm up one level
                if (level < 2) {
                   verticalSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    level += 1;
                    moveLinearSlides();
                }
                attachState = AttachmentState.START;
                break;

            case DOWN: //State for moving arm down one level
                if (level > 0) {
                    verticalSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    level -= 1;
                    moveLinearSlides();
                }
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
        if (gamepad1.left_bumper) {
            leftBackPower /= 3;
            leftFrontPower /= 3;
            rightFrontPower /= 3;
            rightBackPower /= 3;
        } else if (gamepad1.right_bumper) {
            leftBackPower /= 1.2;
            leftFrontPower /= 1.2;
            rightFrontPower /= 1.2;
            rightBackPower /= 1.2;
        } else if (isLoop) {
            leftBackPower /= 1.6;
            leftFrontPower /= 1.6;
            rightFrontPower /= 1.6;
            rightBackPower /= 1.6;
        } else {
            leftBackPower /= 1.6;
            leftFrontPower /= 1.6;
            rightFrontPower /= 1.6;
            rightBackPower /= 1.6;
        }

        //sets the speed of the drive motors
        robot.getLeftBackDrive().setPower(leftBackPower);
        telemetry.addData("motor power of left back motor is", ":", leftBackPower);
        robot.getLeftFrontDrive().setPower(leftFrontPower);
        robot.getRightFrontDrive().setPower(rightFrontPower);
        robot.getRightBackDrive().setPower(rightBackPower);
        telemetry.update();
    }


    public void moveLinearSlides() {

        verticalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        verticalSlideMotor.setTargetPosition(level_ticks[level]);

        // set motors to run to target encoder position and stop with brakes on.
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        if (isLoop) {

            telemetry.addData("Mode", "running");
            telemetry.update();
            if (gamepad2.a) {
                verticalSlideMotor.setPower(-1);
            }

            resetRuntime
                    ();

    while (isLoop && getRuntime() < .6) {
        telemetry.addData("vertical slide pos", verticalSlideMotor.getCurrentPosition() + "  busy=" + verticalSlideMotor.isBusy());
        telemetry.update();
        verticalSlideMotor.setPower(1);

}
    if (verticalSlideMotor.isBusy() != true) {
        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        verticalSlideMotor.setTargetPosition(0);
}
    }
            // wait 5 sec to you can observe the final encoder position.
        verticalSlideMotor.setPower(0.0);
        }
    }


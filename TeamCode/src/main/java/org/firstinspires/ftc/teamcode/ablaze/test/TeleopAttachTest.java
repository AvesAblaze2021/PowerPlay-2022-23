package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ablaze.common.PracticeRobot;
import org.firstinspires.ftc.teamcode.ablaze.teleop.AblazeTeleop;

//Teleop test program for testing attachments
@TeleOp
public class TeleopAttachTest extends OpMode {
    PracticeRobot robot = new PracticeRobot();

    private boolean isLoop = false;
    private int level = 0; //0 = Start, 1 = Ground, 2 = Medium, 3 = High
    private final int[] vertical_level_ticks = {0, 500, 1000, 1500, 1700}; //Tick values for every level
    private final int[] horizontal_level_ticks = {0, 1000};
    private DcMotor verticalSlideMotor;
    private Servo rotationServo;
    private final double motorPower = 0.3;

    private enum AttachmentState{
        START, //Initial State - gamepad monitoring occurs here
        UP, //Moves linear slide up one level
        DOWN, //Moves linear slide down one level
        LEFT, //Rotates arm left
        RIGHT //Rotates arm right
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
                    telemetry.addData("button", "x");
                    telemetry.update();
                    attachState = AttachmentState.LEFT;
                }
                if(gamepad2.b){
                    attachState = AttachmentState.RIGHT;
                }
                break;

            case UP: //State for moving arm up one level
                if(level < vertical_level_ticks.length - 1){
                    level += 1;
                    moveLinearSlides(verticalSlideMotor, vertical_level_ticks);
                }
                attachState = AttachmentState.START;
                break;

            case DOWN: //State for moving arm down one level
                if(level > 0){
                    level -= 1;
                    moveLinearSlides(verticalSlideMotor, vertical_level_ticks);
                }
                attachState = AttachmentState.START;
                break;

            case LEFT:
                rotationServo.setPosition(-0.5);
                telemetry.addData("rotationServo", "moved left");
                telemetry.update();
                attachState = AttachmentState.START;

            case RIGHT:
                rotationServo.setPosition(0.5);
                telemetry.addData("rotationServo", "moved left");
                telemetry.update();
                attachState = AttachmentState.START;
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        isLoop = false;
    }

    public void moveLinearSlides(DcMotor slideMotor, int[] level_ticks){
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setTargetPosition(level_ticks[level]);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(motorPower);
        while (isLoop && slideMotor.isBusy()) {
            telemetry.addData("LFT, RFT", "Running to %7d", level_ticks[level]);
            telemetry.addData("LFP, RFP", "Running at %7d",
                    verticalSlideMotor.getCurrentPosition()
            );
            telemetry.addData("level", level);
            telemetry.update();
        }
        slideMotor.setPower(0.0);
    }
}

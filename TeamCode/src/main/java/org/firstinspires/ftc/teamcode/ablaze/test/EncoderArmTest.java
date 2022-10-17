package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

@Autonomous
public class EncoderArmTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private AblazeRobot ablazeRobot = new AblazeRobot();
    private double motorPower = 1;
    private DcMotor testMotor;

    @Override
    public void runOpMode() {

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Robot");    //
        telemetry.update();
        //sleep(5000);

        // Initialize Robot
        ablazeRobot.initialize(hardwareMap);

        waitForStart();

        // Get the motor to test
        testMotor = ablazeRobot.getArmMotor();

        // Diable  Encoder
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Bring down the arm
        testMotor.setPower(motorPower);
        sleep(750);
        testMotor.setPower(0);
        sleep(300);
        // Lift up arm based on barcode position

        // Enable Encoder
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Stop and rest Encoders
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Identify New Position
        int newPosition = -150;
        double timeLimitMS = 250;

        while (opModeIsActive() ) {

            // Set New Target Position
            testMotor.setTargetPosition(newPosition);

            // Turn On RUN_TO_POSITION
            testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set Speed
            testMotor.setPower(motorPower);

            // reset the timeout time and start motion.
            runtime.reset();

            while (opModeIsActive() &&
                    (runtime.milliseconds() < timeLimitMS) &&
                    (testMotor.isBusy())) {

                // New Position
                int newPos = testMotor.getCurrentPosition();

                telemetry.addData("Runtime: ", runtime.milliseconds());
                telemetry.addData("Position: ", newPos);
                telemetry.update();
            }
            break;

        } // while end

        // Reset Speed
        testMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    } // runOpMode end
} // class end

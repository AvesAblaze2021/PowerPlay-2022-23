package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

@Autonomous
public class AllEncoderTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private AblazeRobot ablazeRobot = new AblazeRobot();
    private double motorPower = .5;

    @Override
    public void runOpMode() {
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Robot");
        telemetry.update();

        // Initialize Robot
        ablazeRobot.initialize(hardwareMap);

        // Enable Encoders

        waitForStart();

        //Stop and reset Encoders

        // Get Ticks
        double ticksPerRevLF = ablazeRobot.getLeftFrontDrive().getMotorType().getTicksPerRev();
        double ticksPerRevRF = ablazeRobot.getRightFrontDrive().getMotorType().getTicksPerRev();
        double ticksPerRevLB = ablazeRobot.getLeftBackDrive().getMotorType().getTicksPerRev();
        double ticksPerRevRB = ablazeRobot.getRightBackDrive().getMotorType().getTicksPerRev();
        telemetry.addData("Ticks Per Rev LF: ", ticksPerRevLF);
        telemetry.addData("Ticks Per Rev RF: ", ticksPerRevRF);
        telemetry.addData("Ticks Per Rev LB: ", ticksPerRevLB);
        telemetry.addData("Ticks Per Rev RB: ", ticksPerRevRB);
        telemetry.update();
        sleep(5000);

        // Calculate New Position
        int newPosition = 500;
        telemetry.addData("New Position: ", newPosition);
        telemetry.update();
        //sleep(5000);
        runtime.reset();

        while (opModeIsActive() ) {

            // Set New Target Position
            ablazeRobot.getLeftFrontDrive().setTargetPosition(newPosition);
            ablazeRobot.getRightFrontDrive().setTargetPosition(newPosition);
            ablazeRobot.getLeftBackDrive().setTargetPosition(newPosition);
            ablazeRobot.getRightBackDrive().setTargetPosition(newPosition);

            // Turn On RUN_TO_POSITION
            ablazeRobot.getLeftFrontDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ablazeRobot.getRightFrontDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ablazeRobot.getLeftBackDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ablazeRobot.getRightBackDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set Speed
            ablazeRobot.getLeftFrontDrive().setPower(Math.abs(motorPower));
            ablazeRobot.getRightFrontDrive().setPower(Math.abs(motorPower));
            ablazeRobot.getLeftBackDrive().setPower(Math.abs(motorPower));
            ablazeRobot.getRightBackDrive().setPower(Math.abs(motorPower));

            // reset the timeout time and start motion.
            runtime.reset();

            while (opModeIsActive() &&
                    (runtime.milliseconds() < 30000)){ //also check if wheels are busy

                // New Position
                int npLF = ablazeRobot.getLeftFrontDrive().getCurrentPosition();
                int npRF = ablazeRobot.getRightFrontDrive().getCurrentPosition();
                int npLB = ablazeRobot.getLeftBackDrive().getCurrentPosition();
                int npRB = ablazeRobot.getRightBackDrive().getCurrentPosition();

                telemetry.addData("Current Position LF: ", npLF);
                telemetry.addData("Current Position RF: ", npRF);
                telemetry.addData("Current Position LB: ", npLB);
                telemetry.addData("Current Position RB: ", npRB);
                telemetry.update();
            }
        } // while end

        // Reset Speed
        ablazeRobot.getLeftFrontDrive().setPower(0);
        ablazeRobot.getRightFrontDrive().setPower(0);
        ablazeRobot.getLeftBackDrive().setPower(0);
        ablazeRobot.getRightBackDrive().setPower(0);

        // Turn off RUN_TO_POSITION
        //ablazeRobot.useEncoderForWheels(true);

    } // runOpMode end
} // class end

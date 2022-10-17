package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

@Autonomous
public class TFODAlgorithm extends LinearOpMode {
    private AblazeRobot ablazeRobot = new AblazeRobot();
    private AblazeTFOD ablazeTfod = new AblazeTFOD();
    private int level;

    @Override
    public void runOpMode(){
        telemetry.addData("initialize", "initialize");
        telemetry.update();

        // Initialize Robot
        ablazeRobot.initialize(hardwareMap);

        // Initialize TFOD
        ablazeTfod.initialize(hardwareMap, ablazeRobot);

        waitForStart();

        telemetry.addData("Wait for Camera to take picture: ", "Please wait...");
        telemetry.update();
        // Give time to camera to start
        sleep(100);
        //Detect barcode position, store in instance variable
        ablazeTfod.detectElement();
        level = ablazeTfod.getLevel();
        telemetry.addData("LEVEL: ", level);
        telemetry.update();

        while(opModeIsActive() ){
            //Detect barcode position, store in instance variable
            ablazeTfod.detectElement();
            this.level = ablazeTfod.getLevel();
            telemetry.addData("Updated LEVEL: ", level);
            telemetry.update();
        }
    }
}

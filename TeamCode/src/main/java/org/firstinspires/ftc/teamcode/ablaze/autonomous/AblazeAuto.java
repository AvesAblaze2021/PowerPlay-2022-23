package org.firstinspires.ftc.teamcode.ablaze.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;

import java.util.Date;
import java.util.List;

@Autonomous
public class AblazeAuto extends LinearOpMode {
    AblazeRobot robot = new AblazeRobot();
    //AblazeTFOD..java tfod = new AblazeTFOD..java();

    String location = "A1";

    @Override
    public void runOpMode() {

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        telemetry.addData("Mode", "running");
        telemetry.update();
    }

    }


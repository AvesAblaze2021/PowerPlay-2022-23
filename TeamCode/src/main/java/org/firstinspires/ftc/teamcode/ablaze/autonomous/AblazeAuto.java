package org.firstinspires.ftc.teamcode.ablaze.autonomous.AblazeAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.TeleOp.SetupClass;

import java.util.Date;
import java.util.List;

public class AblazeAuto {
    SetupClass robot = new SetupClass();
    LinearOpMode startedProgram;
    Telemetry telemetry;
    String location;

    public AblazeAuto ( LinearOpMode startedProgram, String location ) {
        this.startedProgram = startedProgram;
        telemetry = startedProgram.telemetry;
        this.location = location;
    }
  //  private int duckPosition;
   // private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    //private static final String[] LABELS = {
      //      "Ball",
        //    "Cube",
          //  "Duck",
           // "Marker"
    };

   // private static final String VUFORIA_KEY =
           // "AWz6j8L/////AAABmRKkIryTuktLi2mFvZqqaeBDZF67S0ewoDJGMGD7nMiS/el/YAB4BDMhHU9CLQpwfj9cEEkSYB9pZgtsyWTg9q+koX/OUS9w1fDUD2O/ZgUHqvquZ3DgZe+HpsRa3ZcFslOjrqxWO/A7tEYFSJi0OZYLKVD9duT6zYq2OUiT4NJbESkRJvEk0HKmOzIwW395Ujv1uVVxgfaEdIDp4RdMhdI7Fl+ZZ+yKbnoDSnVw/UZHKSg6S/2ZclKQTPZpBmR7wJJp0y4CoSjZZhaukcNSCvsUB6Glr6WajtHP5qDooeWVjmsGi6RRol4h/QlV2sFrLv4ueJS6DPAnOn7oZ9CCeWYavv9cLTYvi6tDB6MuTOsm";
    //private VuforiaLocalizer vuforia;
    //private TFObjectDetector tfod;


    public void runOpMode() throws InterruptedException {
        robot.init(startedProgram.hardwareMap);
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0 / 9.0);
        }
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        telemetry.addData("Mode", "running");
        telemetry.update();
    }


   // private void initVuforia() {
    //    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

      //  parameters.vuforiaLicenseKey = VUFORIA_KEY;
       // parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
       // vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

   /* private void scanDuck() {
        if (tfod != null) {
            long endTime = new Date().getTime() + 3000;

            while (duckPosition == 0 && new Date().getTime() < endTime) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            String thingFound = recognition.getLabel();
                            Float duckLocaion = recognition.getLeft();
                            int markerNumber = 0;
                            if (thingFound.equals("Duck")) {
                                duckPosition = 1;
                                telemetry.addData("I found a ", thingFound);
                                telemetry.addData("I am this percent sure ", recognition.getConfidence());
                                telemetry.addData("Pixels from left to right",
                                        recognition.getLeft());
                                if (duckLocaion < 250) {
                                    markerNumber = 1;
                                    telemetry.addData("It is on the left side", markerNumber);
                                } else {
                                    if (duckLocaion > 750) {
                                        markerNumber = 3;
                                        telemetry.addData("It is on the right side", markerNumber);
                                    } else {
                                        markerNumber = 2;
                                        telemetry.addData("It is in the middle", markerNumber);
*\
                                    }
                                }
                            }
                          //  telemetry.update();
                        }
                        //i++;
                    }
                }
            }
        }
    }


    private void driveit(double frontLeftMotor, double backLeftMotor,
                         double frontRightMotor, double backRightMotor, long sleepTime) {

        robot.backLeftMotor.setPower(backLeftMotor);
        robot.frontLeftMotor.setPower(frontLeftMotor);
        robot.frontRightMotor.setPower(frontRightMotor);
        robot.backRightMotor.setPower(backRightMotor);
        startedProgram.sleep(sleepTime);
        robot.frontLeftMotor.setPower(0.0);
        robot.frontRightMotor.setPower(0.0);
        robot.backLeftMotor.setPower(0.0);
        robot.backRightMotor.setPower(0.0);


    }

    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    //private void initTfod() {
      //  int tfodMonitorViewId = startedProgram.hardwareMap.appContext.getResources().getIdentifier(
        //        "tfodMonitorViewId", "id", startedProgram.hardwareMap.appContext.getPackageName());
        //TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //tfodParameters.minResultConfidence = 0.8f;
        //tfodParameters.isModelTensorFlow2 = true;
        //tfodParameters.inputSize = 320;
        //tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}


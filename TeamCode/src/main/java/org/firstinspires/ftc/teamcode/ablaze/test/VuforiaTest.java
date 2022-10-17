package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
@Autonomous(name="VuforiaTest")
public class VuforiaTest extends LinearOpMode {
    private static final String VUFORIA_KEY =
            "ASgJ37P/////AAABmeZUclRtJkpCrvmEuygWpAwPNksTf+k3zPsoV573qLI1dZpk" +
                    "SHF0eO5ETw9kC0ZvI1ft81RiXDQTvuQ2Nmc5NSjB7TaBkG9QKjeCVbzCCXiAg" +
                    "Gu8DmBF/Xg/7cHcCQqzVHc7RCsbId+BxAJMgz9Yx6pFvkYvGSz/1Fp2KJNw1O3Sg" +
                    "ps/g1Gc0BEln6vpnCjaBouBPNWzEh7vArEih81j0Ek8obttmiNEJ0qKbyefbxWzg" +
                    "ptxEQSyAIqqpNyWFSFngPGYFmhTV7cDahWGi46Jr0OYjJTrPamacGgY4XA8B5UH" +
                    "HLx7BITqp6j3EaZP5VivlYpb3sZLhoKxpwoNgg2E88hcfHkDl28FxLnlrnbMJIL9";

    private VuforiaLocalizer vuforia;
    private WebcamName webcam;
    private VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    private void initVuforia(){
        webcam = hardwareMap.get(WebcamName.class, "VuforiaCam");
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcam;
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    @Override
    public void runOpMode(){
        initVuforia();
        telemetry.addData("Init: ", "Camera");
        telemetry.update();

        waitForStart();

        telemetry.addData("Finished", "Program");
        telemetry.update();
    }
}

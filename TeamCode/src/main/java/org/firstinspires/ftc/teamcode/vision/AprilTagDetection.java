package org.firstinspires.ftc.teamcode.vision;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CommonVariables;
import org.firstinspires.ftc.teamcode.components.Component;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

public class AprilTagDetection extends Component {
    private OpenCvWebcam webcam;
    private AprilTagPipeline pipeline;
    int numFramesWithoutDetection = 0;
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    int detectedTag = 0;


    public AprilTagDetection(CommonVariables commonVariables) {
        super(commonVariables);

    }

    @Override
    public void initialize() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        this.pipeline = new AprilTagPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }


    public void startStream() {
        try {
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        } catch (Exception exception) {
            // Exception closing
        }
    }

    public void stopStream() {
        try {
            webcam.stopStreaming();
        } catch (Exception exception) {
            // Exception closing
        }
    }

    @Override
    public void stop() {
        this.stopStream();
    }

    @Override
    public void update() {
        ArrayList<org.openftc.apriltag.AprilTagDetection> detections = this.pipeline.getDetectionsUpdate();

        // If there's been a new frame...
        if(detections != null) {
            // If we don't see any tags
            if (detections.size() == 0) {
                numFramesWithoutDetection++;

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                    this.pipeline.setDecimation(DECIMATION_LOW);
                }
            }
            // We do see tags!
            else {
                numFramesWithoutDetection = 0;

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                    this.pipeline.setDecimation(DECIMATION_HIGH);
                }

                for (org.openftc.apriltag.AprilTagDetection detection : detections) {
                    telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                    this.detectedTag = detection.id;
                    break;
                }
            }
        }
    }

    public int getPosition() {
        return this.detectedTag;
    }
}

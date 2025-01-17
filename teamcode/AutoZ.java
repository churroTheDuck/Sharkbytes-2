package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Z Start Left score and lvl 1 hang")
public class AutoX extends LinearOpMode {

    // front motors
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;

    // back motors
    DcMotor motorBackLeft;
    DcMotor motorBackRight;

    // arm
    DcMotor motorArm;

    Servo grabServo;
    Servo tiltServo;

    private ElapsedTime     runtime = new ElapsedTime();

    /*
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     *
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     *
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     *
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 4, 3, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initAprilTag();

        // set front motors
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "leftFront");
        motorFrontRight = hardwareMap.get(DcMotor.class, "rightFront");

        // set back motors
        motorBackLeft  = hardwareMap.get(DcMotor.class, "leftBack");
        motorBackRight = hardwareMap.get(DcMotor.class, "rightBack");

        // arm
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");

        tiltServo = hardwareMap.get(Servo.class, "tiltServo");
        grabServo = hardwareMap.get(Servo.class, "grabServo");

        grabServo.setPosition((float) 0.5);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();
        waitForStart();

        double spi = 0.041; // seconds per inch at power set in function
        double spqt = 0.9; // seconds per quarter turn at power set in function
        double flipTime = 0.5; // seconds at half power to change sides
        double flatPos = 90;

        // set up for specimen
        telemetry.addData("Task", "set up for specimen");
        telemetry.update();
        runtime.reset();
        grabServo.setPosition((float) 0.5);
        moveSetPowers(0.0,-0.5,0.0);
        motorArm.setPower((double) -0.5);
        while (opModeIsActive() && (runtime.seconds() < flipTime)) {
            telemetryAprilTag();
            telemetry.update();
        }

        runtime.reset();
        moveSetPowers(0.0,-0.5,0.0);
        motorArm.setPower((double) 0.25);
        while (opModeIsActive() && (runtime.seconds() < 24.0*spi-flipTime)) {
            telemetryAprilTag();
            telemetry.update();
        }

        runtime.reset();
        moveSetPowers(0.0,0.0,-0.5);
        while (opModeIsActive() && (runtime.seconds() < spqt)) {
            telemetryAprilTag();
            telemetry.update();
        }

        runtime.reset();
        tiltServo.setPosition((flatPos+(double) 90.0)/(double) 300.0)
        moveSetPowers(0.0,-0.5,0.0);
        while (opModeIsActive() && (runtime.seconds() < 15.0*spi)) {
            telemetryAprilTag();
            telemetry.update();
        }

        runtime.reset();
        moveSetPowers(0.0,0.0,0.5);
        while (opModeIsActive() && (runtime.seconds() < spqt)) {
            telemetryAprilTag();
            telemetry.update();
        }

        runtime.reset();
        moveSetPowers(0.0,-0.5,0.0);
        while (opModeIsActive() && (runtime.seconds() < 18.0*spi)) {
            telemetryAprilTag();
            telemetry.update();
        }

        // score from touching submersible
        runtime.reset();
        moveSetPowers(0.0,0.5,0.0);
        while (opModeIsActive() && (runtime.seconds() < 8.0*spi)) {
            telemetryAprilTag();
            telemetry.update();
        }

        runtime.reset();
        moveSetPowers(0.0,0.0,0.0);
        motorArm.setPower((double) 0.5);
        while (opModeIsActive() && (runtime.seconds() < flipTime)) {
            telemetryAprilTag();
            telemetry.update();
        }

        runtime.reset();
        grabServo.setPosition((float) 0.2);
        moveSetPowers(0.0,0.5,0.0);
        motorArm.setPower((double) -0.4);
        while (opModeIsActive() && (runtime.seconds() < flipTime)) {
            telemetryAprilTag();
            telemetry.update();
        }

        runtime.reset();
        moveSetPowers(0.0,-0.5,0.0);
        motorArm.setPower((double) 0.25);
        while (opModeIsActive() && (runtime.seconds() < flipTime+3.0*spi)) {
            telemetryAprilTag();
            telemetry.update();
        }

        runtime.reset();
        moveSetPowers(0.0,0.5,0.0);
        while (opModeIsActive() && (runtime.seconds() < 6.0*spi)) {
            telemetryAprilTag();
            telemetry.update();
        }

        // going to hang
        runtime.reset();
        moveSetPowers(0.0,0.0,-0.5);
        while (opModeIsActive() && (runtime.seconds() < spqt)) {
            telemetryAprilTag();
            telemetry.update();
        }

        runtime.reset();
        tiltServo.setPosition((flatPos)/(double) 300.0);
        moveSetPowers(0.0,0.5,0.0);
        while (opModeIsActive() && (runtime.seconds() < 36.0*spi)) {
            telemetryAprilTag();
            telemetry.update();
        }

        runtime.reset();
        moveSetPowers(0.0,0.0,-0.5);
        while (opModeIsActive() && (runtime.seconds() < spqt)) {
            telemetryAprilTag();
            telemetry.update();
        }

        runtime.reset();
        moveSetPowers(0.0,0.5,0.0);
        while (opModeIsActive() && (runtime.seconds() < 30.0*spi)) {
            telemetryAprilTag();
            telemetry.update();
        }

        runtime.reset();
        moveSetPowers(0.0,0.0,-0.5);
        while (opModeIsActive() && (runtime.seconds() < spqt)) {
            telemetryAprilTag();
            telemetry.update();
        }

        runtime.reset();
        moveSetPowers(0.0,-0.5,0.0);
        while (opModeIsActive() && (runtime.seconds() < 18.0*spi)) {
            telemetryAprilTag();
            telemetry.update();
        }

        runtime.reset();
        moveSetPowers(0.0,0.0,0.0);
        motorArm.setPower((double) 0.5);
        while (opModeIsActive() && (runtime.seconds() < flipTime)) {
            telemetryAprilTag();
            telemetry.update();
        }


        motorArm.setPower((double) 0.0);
        moveSetPowers(0.0,0.0,0.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        visionPortal.close();
        sleep(1000);
    }




    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    private int telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        double minDist = 0.0;
        int id = -1;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                id = detection.id;
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

        return id;

    }   // end method telemetryAprilTag()

    private void moveSetPowers(double moveX, double moveY, double rot) {
        double power = 1;

        motorFrontLeft.setPower((moveX - moveY - rot)*power);
        motorFrontRight.setPower((-moveX - moveY + rot)*power);
        motorBackLeft.setPower((-moveX - moveY - rot)*power);
        motorBackRight.setPower((moveX - moveY + rot)*power);
    }
}

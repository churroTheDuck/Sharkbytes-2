package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;


// driving handler
public class DrivingHandler {
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

    boolean grab;
    boolean prevB;
    boolean prevA;
    boolean prevX;
    boolean holdArm;
    boolean scoreSpecimen;

    double servoPos;

    private ElapsedTime     runtime = new ElapsedTime();

    // initialization
    public DrivingHandler(HardwareMap hardwareMap) {
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

        grab = false;
        prevB = false;
        prevA = false;
        scoreSpecimen = false;

        servoPos = 0.0;

    }

    // gameplay loop
    public void loop(Gamepad gamepad1, Telemetry telemetry) {
        float leftStickX  = gamepad1.left_stick_x;
        float leftStickY  = gamepad1.left_stick_y;
        float rightStickX = gamepad1.right_stick_x;
        float rightStickY = gamepad1.right_stick_y;
        float leftTrigger = gamepad1.left_trigger;
        float rightTrigger = gamepad1.right_trigger;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        double arm = 0.0;

        double spi = 0.041; // seconds per inch at power set in function
        double spqt = 0.9; // seconds per quarter turn at power set in function
        double flipTime = 0.5; // seconds at half power to change sides
        double flatPos = 90;

        moveSetPowers(-leftStickX, -leftStickY, rightStickX);

        if (rightStickY>0) {
            arm = -(rightStickY)*(rightStickY) / (float) 2.0 + (float) 0.0;
        } else {
            arm = (rightStickY)*(rightStickY) / (float) 2.0 + (float) 0.0;
        }



        // x to toggle arm hold
        if (gamepad1.x & !prevX) {
            holdArm = !holdArm;
        }
        if (holdArm) {
            arm = (float) 0.25;
        }

        // b to toggle grab
        if (gamepad1.b & !prevB) {
            grab = !grab;
        }
        if (grab) {
            grabServo.setPosition((float) 0.5);
        } else {
            grabServo.setPosition((float) 0.2);
        }

        // left and right trigger for tiltServo

        tiltServo.setPosition((flatPos - (double) 90.0 * (rightTrigger-leftTrigger))/(double) 300.0);

        // a to run specimen sequence
        if (gamepad1.a & !prevA) {
            scoreSpecimen = true;
            runtime.reset();
        }
        if (scoreSpecimen) {
            double timeTracker = 0.0;
            timeTracker += 8.0 * spi;
            if (runtime.seconds() < timeTracker) { // change to how far back is needed
                tiltServo.setPosition((flatPos+(double) 90.0)/(double) 300.0); // set to angle for dunk
                grabServo.setPosition((double) 0.0); // set to closed position
                arm = (float) -0.25;
                moveSetPowers(0.0,-0.5,0.0);
            }
            timeTracker += 0.8;
            if (runtime.seconds() < timeTracker) {
                tiltServo.setPosition((flatPos+(double) 90.0)/(double) 300.0);
                grabServo.setPosition((double) 0.0);
                arm = (float) -0.5;
                moveSetPowers(0.0,0.0,0.0);
            }
            timeTracker += 0.4;
            if (runtime.seconds() < timeTracker) {
                tiltServo.setPosition((flatPos+(double) 90.0)/(double) 300.0);
                grabServo.setPosition((double) 0.2); // set to open claw
                arm = (float) -0.1;
                moveSetPowers(0.0,0.0,0.0);
            }
            timeTracker += 6.0*spi;
            if (runtime.seconds() < timeTracker) {
                tiltServo.setPosition(flatPos / (double) 300.0);
                grabServo.setPosition((double) 0.2); // set to open claw
                arm = (float) 0.25;
                moveSetPowers(0.0,0.5,0.0);
            }
            timeTracker += 0.8;
            if (runtime.seconds() < timeTracker) {
                tiltServo.setPosition(flatPos / (double) 300.0);
                grabServo.setPosition((double) 0.2);
                arm = (float) 0.5;
                moveSetPowers(0.0, 0.0, 0.0);
            }
            if (runtime.seconds() > timeTracker) {
                grab = false;
                scoreSpecimen = false;
            }
        }
        // end of a for specimen sequence

        motorArm.setPower(arm);
        telemetry.addData("arm", arm);
        telemetry.addData("grab", grab);
        prevB = gamepad1.b;
        prevA = gamepad1.a;
        prevX = gamepad1.x;
    }

    private void moveSetPowers(double moveX, double moveY, double rot) {
        double power = 1;

        double frontLeft = (moveX - moveY - rot)*power;
        double frontRight = (-moveX - moveY + rot)*power;
        double backLeft = (-moveX - moveY - rot)*power;
        double backRight = (moveX - moveY + rot)*power;

        double max = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
        max = Math.max(max, Math.abs(backLeft));
        max = Math.max(max, Math.abs(backRight));

        if (max > 1.0) {
            frontLeft /= max;
            frontRight /= max;
            backLeft /= max;
            backRight /= max;
        }



        motorFrontLeft.setPower(frontLeft);
        motorFrontRight.setPower(frontRight);
        motorBackLeft.setPower(backLeft);
        motorBackRight.setPower(backRight);
    }
}




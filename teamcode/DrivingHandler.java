package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
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
    
    CRServo grabServo;

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
        
        grabServo = hardwareMap.get(CRServo.class, "grabServo");
    }

    // gameplay loop
    public void loop(Gamepad gamepad1) {
        float leftStickX  = gamepad1.left_stick_x;
        float leftStickY  = gamepad1.left_stick_y;
        float rightStickX = gamepad1.right_stick_x;
        float rightStickY = gamepad1.right_stick_y;
        float leftTrigger = gamepad1.left_trigger;
        float rightTrigger = gamepad1.right_trigger;

        float moveX = leftStickX;
        float moveY = rightTrigger-leftTrigger;
        float rot = rightStickX;
        float arm = rightStickY / (float) 1.75;
        
        
        double power = 1;
 
        // set motor powers
        double frontLeft = (moveX - moveY + rot)*power;
        double frontRight = (-moveX - moveY - rot)*power;
        double backLeft = (-moveX - moveY + rot)*power;
        double backRight = (moveX - moveY - rot)*power;

        double max = Math.max(frontLeft, frontRight);
        max = Math.max(max, backLeft);
        max = Math.max(max, backRight);

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

        // arm
        motorArm.setPower(arm);
        
        grabServo.setPower(-leftStickY);

    }
}

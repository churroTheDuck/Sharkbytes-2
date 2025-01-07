package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
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
    
    CRServo grabServo;
    
    boolean holdArm;
    boolean prevB;

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
        
        holdArm = false;
        prevB = false;
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

        float rot = rightStickX;
        float arm = rightTrigger - leftTrigger / (float) 1.0 + (float) 0.0;
        
        if (gamepad1.b & !prevB) {
            holdArm = !holdArm;
        }
        
        if (holdArm) {
            arm = (float) 0.24;
        }
        
        telemetry.addData("arm", arm);
        
        double power = 1;
 
        // set motor powers
        double frontLeft = (leftStickY+leftStickX)*power;
        double frontRight = (rightStickY-leftStickX)*power;
        double backLeft = (leftStickY-leftStickX)*power;
        double backRight = (rightStickY+leftStickX)*power;
        
        
        grabServo.setPower((rightStickX)*power);
        float y = gamepad1.y ? 1 : 0;
        float a = gamepad1.a ? 1 : 0;
        double servoSpeed = y - a;
        //grabServo.setPower(servoSpeed);
        telemetry.addData("servo", servoSpeed);
        
        // if (gamepad1.y) {
        //      grabServo.setPower(1.0);
        // } else if (gamepad1.a) {
        //      grabServo.setPower(-1.0);
        // } else {
        //      grabServo.setPower(0.0);
        // }

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
        
        prevB = gamepad1.b;
    }
}

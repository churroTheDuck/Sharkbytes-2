package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
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

    // initialization
    public DrivingHandler(HardwareMap hardwareMap) {
        // set front motors
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");

        // set back motors
        motorBackLeft  = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");

        // arm
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
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
        float arm = rightStickY;

        
        double power = 1;

        // set motor powers
        motorFrontLeft.setPower(Math.min(moveX   - moveY + rot, 1) * power);
        motorFrontRight.setPower(Math.min(-moveX - moveY - rot, 1) * power);
        motorBackLeft.setPower(Math.min(-moveX   - moveY + rot, 1) * power);
        motorBackRight.setPower(Math.min(moveX   - moveY - rot, 1) * power);

        // arm
        motorArm.setPower(arm);
        
        // float frontLeft = 0;
        // float frontRight = 0;
        // float backLeft = 0;
        // float backRight = 0;
        
        // System.out.println(gamepad1.dpad_up);

        // dpad control

    }
}

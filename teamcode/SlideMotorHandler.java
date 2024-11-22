package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;

// slide motor handler
public class SlideMotorHandler {
    // linear slide motor
    DcMotor slideMotor;

    // initialization
    public SlideMotorHandler(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
    }

    // gameplay loop
    public void loop(Gamepad gamepad1) {
        double motorPower = 0;

        // bumpers control
        if (gamepad1.left_bumper) {
            motorPower -= 1;
        }
        if (gamepad1.right_bumper) {
            motorPower += 1;
        }

        // set linear slide motor power
        slideMotor.setPower(motorPower);
    }
}

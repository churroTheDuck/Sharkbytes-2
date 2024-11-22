package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

// op mode for testing
@TeleOp
public class Testing extends LinearOpMode {
    // hardware motor
    private DcMotor motor;
    private int motorPower;

    @Override
    public void runOpMode() {
        // set hardware motor
        motor = hardwareMap.get(DcMotor.class, "motor");

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // set motor power
            motorPower = gamepad1.a ? 1 : 0;

            if (motorPower > 0) {
                telemetry.addData("motor", "spinning");
            } else {
                telemetry.addData("motor", "stopped");
            }

            motor.setPower(motorPower);

            // update status
            telemetry.addData("status", "running");
            telemetry.update();
        }
    }
}

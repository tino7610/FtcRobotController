package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Test TeleOp", group="Linear Opmode")
//@Disabled

public class _7610_TestContServoRamp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo ramp = null;


    @Override
    public void runOpMode() {
        ramp = hardwareMap.get(CRServo.class, "Ramp"); //ASK WHAT THE NAME IS
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        ramp.setDirection(DcMotor.Direction.REVERSE);
        ramp.setPower(0.5);
        sleep(2000);
        ramp.setPower(0);

    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Test AutonomousV2", group="Linear Opmode")
//@Disabled
public class _7610_TestAutonomousV2 extends LinearOpMode{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fLDrive = null;
    private DcMotor fRDrive = null;
    private DcMotor bLDrive = null;
    private DcMotor bRDrive = null;

    private DcMotor outtake = null;
    private DcMotor cBelt = null;

    private long startTime;

    double outPower;
    double cBeltPower;

    private CRServo armElbow = null;
    private Servo armWrist = null;
    private AnalogInput analog;

    private double ticksPerRevolution = 28 * 40 / 2.6; //28 ticks for motor, x40 for gearbox, 10:26 gear ratio so /2.6
    private double ticksPerInch = ticksPerRevolution / (4 * Math.PI); //four-inch diameter

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fLDrive  = hardwareMap.get(DcMotor.class, "LeftFront");
        fRDrive = hardwareMap.get(DcMotor.class, "RightFront");
        bLDrive  = hardwareMap.get(DcMotor.class, "LeftRear");
        bRDrive = hardwareMap.get(DcMotor.class, "RightRear");

        armElbow = hardwareMap.get(CRServo.class, "wobbleservo");
        armWrist = hardwareMap.get(Servo.class, "armWrist");

        analog = hardwareMap.get(AnalogInput.class, "wobblepot");
        outtake = hardwareMap.get(DcMotor.class, "shooter");
        cBelt = hardwareMap.get(DcMotor.class,"belt");

        outtake.setDirection(DcMotor.Direction.FORWARD);
        cBelt.setDirection(DcMotor.Direction.FORWARD);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        fLDrive.setDirection(DcMotor.Direction.REVERSE);
        fRDrive.setDirection(DcMotor.Direction.FORWARD);
        bLDrive.setDirection(DcMotor.Direction.REVERSE);
        bRDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)

        //move left in front of starter stack to shoot rings and identify how many rings are in stack

        fLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fLDrive.setTargetPosition(-9 * (int)ticksPerInch);
        fRDrive.setTargetPosition(9 * (int)ticksPerInch);
        bLDrive.setTargetPosition(9 * (int)ticksPerInch);
        bRDrive.setTargetPosition(-9 * (int)ticksPerInch);

        fLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fLDrive.setPower(0.5);
        fRDrive.setPower(0.5);
        bLDrive.setPower(0.5);
        bRDrive.setPower(0.5);

        //shoot the preloaded rings

        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // setting motor powers

        waitForStart();
            startTime = System.currentTimeMillis();
            cBeltPower = -1.0;
            if(System.currentTimeMillis() < startTime + 500)
                cBeltPower = -1.0;
            else{
                cBeltPower = 1.0;
                outPower = -1.0;
                }
            if(System.currentTimeMillis() > startTime + 7500)

            outPower = 0.0;
            cBeltPower = 0.0;

        //move back to start line to pick up wobble goal

        while(fLDrive.isBusy()) {

            telemetry.addData("fLDrive", fLDrive.getCurrentPosition());
            telemetry.addData("fRDrive", fRDrive.getCurrentPosition());
            telemetry.addData("bLDrive", bLDrive.getCurrentPosition());
            telemetry.addData("bRDrive", bRDrive.getCurrentPosition());

        }

        fLDrive.setPower(0.0);
        fRDrive.setPower(0.0);
        bLDrive.setPower(0.0);
        bRDrive.setPower(0.0);

        fLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fLDrive.setTargetPosition(17 * (int)ticksPerInch);
        fRDrive.setTargetPosition(-17 * (int)ticksPerInch);
        bLDrive.setTargetPosition(-17 * (int)ticksPerInch);
        bRDrive.setTargetPosition(17 * (int)ticksPerInch);

        //pick up wobble goal

        armWrist.setPosition(0.5);

        while (analog.getVoltage() > 0.369) armElbow.setPower(0.5);
        armElbow.setPower(0);

        armWrist.setPosition(0.0);

        //move to Target Zone A

        fLDrive.setTargetPosition(3 * (int)ticksPerInch);
        fRDrive.setTargetPosition(-3 * (int)ticksPerInch);
        bLDrive.setTargetPosition(-3 * (int)ticksPerInch);
        bRDrive.setTargetPosition(3 * (int)ticksPerInch);

        fLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fLDrive.setPower(0.5);
        fRDrive.setPower(0.5);
        bLDrive.setPower(0.5);
        bRDrive.setPower(0.5);

        while(fLDrive.isBusy()) {

            telemetry.addData("fLDrive", fLDrive.getCurrentPosition());
            telemetry.addData("fRDrive", fRDrive.getCurrentPosition());
            telemetry.addData("bLDrive", bLDrive.getCurrentPosition());
            telemetry.addData("bRDrive", bRDrive.getCurrentPosition());

        }

        fLDrive.setPower(0.0);
        fRDrive.setPower(0.0);
        bLDrive.setPower(0.0);
        bRDrive.setPower(0.0);

        fLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Starting robot from the right-most start line in front of starter stack
        //if # objects == 0 rings
        //For going to the target zone A
        fLDrive.setTargetPosition(41 * (int)ticksPerInch);
        fRDrive.setTargetPosition(41 * (int)ticksPerInch);
        bLDrive.setTargetPosition(41 * (int)ticksPerInch);
        bRDrive.setTargetPosition(41 * (int)ticksPerInch);

        fLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fLDrive.setPower(0.5);
        fRDrive.setPower(0.5);
        bLDrive.setPower(0.5);
        bRDrive.setPower(0.5);

        while(fLDrive.isBusy()) {

            telemetry.addData("fLDrive", fLDrive.getCurrentPosition());
            telemetry.addData("fRDrive", fRDrive.getCurrentPosition());
            telemetry.addData("bLDrive", bLDrive.getCurrentPosition());
            telemetry.addData("bRDrive", bRDrive.getCurrentPosition());

        }

        fLDrive.setPower(0.0);
        fRDrive.setPower(0.0);
        bLDrive.setPower(0.0);
        bRDrive.setPower(0.0);

        fLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //release wobble goal into Target Zone A

        armWrist.setPosition(0.5);

        //Drive robot to park on launch line (hard coded)

        fLDrive.setTargetPosition(36 * (int)ticksPerInch);
        fRDrive.setTargetPosition(-36 * (int)ticksPerInch);
        bLDrive.setTargetPosition(-36 * (int)ticksPerInch);
        bRDrive.setTargetPosition(36 * (int)ticksPerInch);

        fLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fLDrive.setPower(0.5);
        fRDrive.setPower(0.5);
        bLDrive.setPower(0.5);
        bRDrive.setPower(0.5);

        while(fLDrive.isBusy()) {

            telemetry.addData("fLDrive", fLDrive.getCurrentPosition());
            telemetry.addData("fRDrive", fRDrive.getCurrentPosition());
            telemetry.addData("bLDrive", bLDrive.getCurrentPosition());
            telemetry.addData("bRDrive", bRDrive.getCurrentPosition());

        }

        fLDrive.setPower(0.0);
        fRDrive.setPower(0.0);
        bLDrive.setPower(0.0);
        bRDrive.setPower(0.0);

        fLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fLDrive.setTargetPosition(24 * (int)ticksPerInch);
        fRDrive.setTargetPosition(24 * (int)ticksPerInch);
        bLDrive.setTargetPosition(24 * (int)ticksPerInch);
        bRDrive.setTargetPosition(24 * (int)ticksPerInch);

        fLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fLDrive.setPower(0.5);
        fRDrive.setPower(0.5);
        bLDrive.setPower(0.5);
        bRDrive.setPower(0.5);

        while(fLDrive.isBusy()) {

            telemetry.addData("fLDrive", fLDrive.getCurrentPosition());
            telemetry.addData("fRDrive", fRDrive.getCurrentPosition());
            telemetry.addData("bLDrive", bLDrive.getCurrentPosition());
            telemetry.addData("bRDrive", bRDrive.getCurrentPosition());

        }

        fLDrive.setPower(0.0);
        fRDrive.setPower(0.0);
        bLDrive.setPower(0.0);
        bRDrive.setPower(0.0);

        fLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //sleeps if all else fails :)

        sleep(30000);

    }

}






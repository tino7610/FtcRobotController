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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@Autonomous(name="Test Autonomous", group="Linear Opmode")
//@Disabled
public class _7610_TestAutonomous extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fLDrive = null;
    private DcMotor fRDrive = null;
    private DcMotor bLDrive = null;
    private DcMotor bRDrive = null;
    private DcMotor intake = null;

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

        fLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Starting robot from the left-most start line

        //For going to the first target zone (closest)

        fLDrive.setTargetPosition(60 * (int)ticksPerInch);
        fRDrive.setTargetPosition(60 * (int)ticksPerInch);
        bLDrive.setTargetPosition(60 * (int)ticksPerInch);
        bRDrive.setTargetPosition(60 * (int)ticksPerInch);

        fLDrive.setTargetPosition(-24 * (int)ticksPerInch);
        fRDrive.setTargetPosition(24 * (int)ticksPerInch);
        bLDrive.setTargetPosition(24 * (int)ticksPerInch);
        bRDrive.setTargetPosition(-24 * (int)ticksPerInch);

        //For going to the second target zone (middle) (the one straight ahead of the robot start line)

        fLDrive.setTargetPosition(84 * (int)ticksPerInch);
        fRDrive.setTargetPosition(84 * (int)ticksPerInch);
        bLDrive.setTargetPosition(84 * (int)ticksPerInch);
        bRDrive.setTargetPosition(84 * (int)ticksPerInch);

        //For going to the third target zone (farthest one)

        fLDrive.setTargetPosition(108 * (int)ticksPerInch);
        fRDrive.setTargetPosition(108 * (int)ticksPerInch);
        bLDrive.setTargetPosition(108 * (int)ticksPerInch);
        bRDrive.setTargetPosition(108 * (int)ticksPerInch);

        fLDrive.setTargetPosition(-24 * (int)ticksPerInch);
        fRDrive.setTargetPosition(24 * (int)ticksPerInch);
        bLDrive.setTargetPosition(24 * (int)ticksPerInch);
        bRDrive.setTargetPosition(-24 * (int)ticksPerInch);

        fLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        fLDrive.setPower(0.5);
        fRDrive.setPower(0.5);
        bLDrive.setPower(0.5);
        bRDrive.setPower(0.5);

        //Returning back to start line to park robot

        fLDrive.setTargetPosition(-60 * (int)ticksPerInch);
        fRDrive.setTargetPosition(-60 * (int)ticksPerInch);
        bLDrive.setTargetPosition(-60 * (int)ticksPerInch);
        bRDrive.setTargetPosition(-60 * (int)ticksPerInch);

        fLDrive.setTargetPosition(24 * (int)ticksPerInch);
        fRDrive.setTargetPosition(-24 * (int)ticksPerInch);
        bLDrive.setTargetPosition(-24 * (int)ticksPerInch);
        bRDrive.setTargetPosition(24 * (int)ticksPerInch);

        //Coming back from second target zone (middle) (the one straight ahead of the robot start line)

        fLDrive.setTargetPosition(-84 * (int)ticksPerInch);
        fRDrive.setTargetPosition(-84 * (int)ticksPerInch);
        bLDrive.setTargetPosition(-84 * (int)ticksPerInch);
        bRDrive.setTargetPosition(-84 * (int)ticksPerInch);

        //Coming back from third target zone (farthest one)

        fLDrive.setTargetPosition(-108 * (int)ticksPerInch);
        fRDrive.setTargetPosition(-108 * (int)ticksPerInch);
        bLDrive.setTargetPosition(-108 * (int)ticksPerInch);
        bRDrive.setTargetPosition(-108 * (int)ticksPerInch);

        fLDrive.setTargetPosition(24 * (int)ticksPerInch);
        fRDrive.setTargetPosition(-24 * (int)ticksPerInch);
        bLDrive.setTargetPosition(-24 * (int)ticksPerInch);
        bRDrive.setTargetPosition(24 * (int)ticksPerInch);

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
        /*fLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fLDrive.setTargetPosition(-1440);
        fRDrive.setTargetPosition(-1440);
        bLDrive.setTargetPosition(-1440);
        bRDrive.setTargetPosition(-1440);

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

        fLDrive.setTargetPosition(-1440);
        fRDrive.setTargetPosition(1440);
        bLDrive.setTargetPosition(1440);
        bRDrive.setTargetPosition(-1440);

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

        fLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fLDrive.setTargetPosition(1440);
        fRDrive.setTargetPosition(-1440);
        bLDrive.setTargetPosition(-1440);
        bRDrive.setTargetPosition(1440);

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

         */

    }

}






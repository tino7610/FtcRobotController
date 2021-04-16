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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test TeleOp", group="Iterative Opmode")
//@Disabled
public class _7610_TestTeleOp extends OpMode
{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fLDrive = null;
    private DcMotor fRDrive = null;
    private DcMotor bLDrive = null;
    private DcMotor bRDrive = null;
    private DcMotor intake = null;
    private DcMotor outtake = null;
    private DcMotor cBelt = null;

    private double voltageReading = 0;

    //servos
    private CRServo ramp = null;
    private CRServo armElbow = null;
    private Servo armWrist = null;
    private AnalogInput analog;


    //ramp
    //private boolean aPressed = false;


    //outtake
    private long startTime;
    private boolean rBumperPressed = false;
    private double time = 0.0;

    //wrist
    private double armWristPos = 0;
    private boolean bPressed = false;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fLDrive  = hardwareMap.get(DcMotor.class, "LeftFront");
        fRDrive = hardwareMap.get(DcMotor.class, "RightFront");
        bLDrive  = hardwareMap.get(DcMotor.class, "LeftRear");
        bRDrive = hardwareMap.get(DcMotor.class, "RightRear");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotor.class, "shooter");
        cBelt = hardwareMap.get(DcMotor.class,"belt");

        ramp = hardwareMap.get(CRServo.class, "Ramp"); //ASK WILLIAMS WHAT THIS IS NAMED
        armElbow = hardwareMap.get(CRServo.class, "wobbleservo");
        armWrist = hardwareMap.get(Servo.class, "armWrist");

        analog = hardwareMap.get(AnalogInput.class, "wobblepot");


        fLDrive.setDirection(DcMotor.Direction.REVERSE);
        fRDrive.setDirection(DcMotor.Direction.FORWARD);
        bLDrive.setDirection(DcMotor.Direction.REVERSE);
        bRDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setDirection(DcMotor.Direction.FORWARD);
        cBelt.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double fLPower;
        double fRPower;
        double bLPower;
        double bRPower;
        double inPower;
        double outPower;
        double cBeltPower;

        double elbowPower;
        double rampPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;

        cBeltPower = 0.0;

        if(gamepad1.left_bumper){
            inPower = 0.5;
            cBeltPower = 1.0;
        } else {
            inPower = 0.0;
        }

        //shootingCode
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // setting motor powers
        if(gamepad1.right_bumper){

            outPower = 0.0;

            if(!rBumperPressed) {

                rBumperPressed = true;
                startTime = System.currentTimeMillis();
                cBeltPower = -1.0;


            }
            else {

                if(System.currentTimeMillis() < startTime + 500) cBeltPower = -1.0;
                else {

                    cBeltPower = 1.0;
                    outPower = -1.0;

                }

            }

        }

        else {
            outPower = 0.0;
            rBumperPressed = false;
        }

        /*if(gamepad1.a && rampPos == 0) {
            rampPos = 1;
        }

        /*if(gamepad1.a && !aPressed) {
            if(rampPos == 1) rampPos = 0;
            else if (rampPos == 0) rampPos = 1;
            aPressed = true;
        }
        else if (!gamepad1.a) aPressed = false;



*/

        //ramp code because we don't have potentiometer values lol
        if(gamepad1.a) rampPower = 0.5;
        else if(gamepad1.b) rampPower = -0.5;
        else rampPower = 0;

        //armCode
        voltageReading = analog.getVoltage();

        if ((analog.getVoltage() >= 0.369 && gamepad2.left_stick_y < 0) || (analog.getVoltage() <= 2.25 && gamepad2.left_stick_y > 0))
            elbowPower = Range.clip(gamepad2.left_stick_y, -1.0, 1.0);
        else elbowPower = 0;

        if(gamepad2.b && !bPressed) {
            if(armWristPos == 0.5) armWristPos = 0;
            else armWristPos = 0.5;
            bPressed = true;
        }
        else if (!gamepad2.b) bPressed = false;

        //driveCode
        fLPower   = Range.clip(y + x + 0.7 * r, -0.5, 0.5) ;
        fRPower   = Range.clip(y - x - 0.7 * r, -0.5, 0.5) ;
        bLPower   = Range.clip(y - x + 0.7 * r, -0.5, 0.5) ;
        bRPower   = Range.clip(y + x - 0.7 * r, -0.5, 0.5) ;

        // Send calculated power to wheels
        fLDrive.setPower(fLPower);
        fRDrive.setPower(fRPower);
        bLDrive.setPower(bLPower);
        bRDrive.setPower(bRPower);
        armElbow.setPower(elbowPower);
        armWrist.setPosition(armWristPos);
        ramp.setPower(rampPower);
        intake.setPower(inPower);
        outtake.setPower(outPower);
        cBelt.setPower(cBeltPower);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left front (%.2f), right front (%.2f), left back (%.2f), right back (%.2f)",
                fLPower, fRPower, bLPower, bRPower);


        telemetry.addData("Intake", "Power: " + inPower);
        telemetry.addData("Outtake", "Power: " + outPower);
        telemetry.addData("ConveyorBelt","Power:" + cBeltPower);
        telemetry.addData("Voltage Reading: ", analog.getVoltage());
        telemetry.addData("Elbow ", "Power: " + elbowPower);
        telemetry.addData("Ramp ", "Power: " + rampPower);
        //need telemetry for others
        //telemetry.addData("Ramp", "Position: " + rampPos);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

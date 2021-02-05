package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
public class _7610_TestOuttake extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor outtake = null;
    private DcMotor cBelt = null;

    private boolean rBumperPressed = false;
    private double time = 0.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        outtake = hardwareMap.get(DcMotor.class, "shooter");
        cBelt = hardwareMap.get(DcMotor.class,"belt");

        cBelt.setDirection(DcMotor.Direction.FORWARD);

        outtake.setDirection(DcMotor.Direction.REVERSE);


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
        double outPower;
        double cBeltPower;


        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.


        // Send calculated power to wheels


        // Show the elapsed game time and wheel power.

        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // setting motor powers
        if(gamepad1.right_bumper){
            outPower = 1.0;
            cBeltPower = 1.0;
        } else {
            outPower = 0.0;
            cBeltPower = 0.0;
        }
        // reversing conveyor belt for some time to get momentum and then going forward into outtake
        if(gamepad1.right_bumper){
            long startTime = System.currentTimeMillis();
            long endTime = startTime;
            long count = startTime + 500;
            while (endTime <= count) {
                cBelt.setDirection(DcMotor.Direction.REVERSE);
                outPower = 1.0;
                endTime = System.currentTimeMillis();
            while (endTime < count) {
                cBelt.setDirection(DcMotor.Direction.FORWARD);
                outPower = 1.0;
                endTime = System.currentTimeMillis();
                }
            }

        /*if(gamepad1.right_bumper) {

            cBeltPower = 1.0;

            if (!rBumperPressed) {

                rBumperPressed = true;
                time = getRuntime();
                cBelt.setDirection(DcMotor.Direction.REVERSE);

            }

            else if (getRuntime() - time == 500) {

                cBelt.setDirection(DcMotor.Direction.FORWARD);
                outPower = 1.0;

            else if (!gamepad1.right_bumper) rBumperPressed = false;
            }

        public class TestTimer1 {
	        public static void main(String[] args) {
		        long startTime = System.currentTimeMillis();
		        long endTime = startTime;
		        long count = startTime + 500;
		        long buttonNotPressed = count + 500;
		        System.out.print(startTime + " " + endTime + " " + count);
		        while (endTime <= count) {
			        System.out.println("REVERSE!!");
			        endTime = System.currentTimeMillis();
	    	}
		    System.out.println(endTime);
		    while (endTime < buttonNotPressed) {
			    System.out.println("FORWARD;)");
			    endTime = System.currentTimeMillis();
		    }


	}
}

         */



        }


        outtake.setPower(outPower);
        cBelt.setPower(cBeltPower);

        telemetry.addData("Outtake", "Power: " + outPower);
        telemetry.addData("ConveyorBelt","Power:" + cBeltPower);


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

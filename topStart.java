package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "JPL_TopStart_FOUR_MOTORS", group = "JPL")
public class fourMotors_TopStart extends LinearOpMode {
    
    private DcMotorEx myMotor0 = null;
    private DcMotorEx myMotor1 = null;
    private DcMotorEx myMotor2 = null;
    private DcMotorEx myMotor3 = null;
    
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        int UP_POSITION = -545; 
        int DOWN_POSITION = 100;
        double ratio_offset = 1.5;

        double LIFT_VEL = 600; 
        double LOWER_VEL = 350;
        
        // Define the distance from Top to Bottom for the first move
        // Since UP was -545, going down from top is +545
        int INITIAL_DROP_DISTANCE = 545; 

        myMotor0 = hardwareMap.get(DcMotorEx.class, "motor0");
        myMotor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        myMotor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        myMotor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        
        myMotor0.setDirection(DcMotorSimple.Direction.REVERSE);
        myMotor1.setDirection(DcMotorSimple.Direction.REVERSE); 
        myMotor2.setDirection(DcMotorSimple.Direction.FORWARD); 
        myMotor3.setDirection(DcMotorSimple.Direction.FORWARD);
        
        int tolerance = 15;
        myMotor0.setTargetPositionTolerance(tolerance);
        myMotor1.setTargetPositionTolerance(tolerance);
        myMotor2.setTargetPositionTolerance(tolerance);
        myMotor3.setTargetPositionTolerance(tolerance);
        
        // --- IMPORTANT INIT STEP ---
        // Manually hold the arm against the TOP STOPPER before pressing Init!
        myMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        myMotor0.setTargetPosition(0);
        myMotor1.setTargetPosition(0);
        myMotor2.setTargetPosition(0);
        myMotor3.setTargetPosition(0);

        myMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // PIDF Setup
        PIDFCoefficients pidf40 = myMotor0.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf60 = myMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        
        PIDFCoefficients liftPIDF40 = new PIDFCoefficients(pidf40.p, pidf40.i, pidf40.d, pidf40.f + 25);
        PIDFCoefficients liftPIDF60 = new PIDFCoefficients(pidf60.p, pidf60.i, pidf60.d, pidf60.f + 15);

        PIDFCoefficients downPIDF40 = new PIDFCoefficients(pidf40.p, pidf40.i, pidf40.d, pidf40.f);
        PIDFCoefficients downPIDF60 = new PIDFCoefficients(pidf60.p, pidf60.i, pidf60.d, pidf60.f);
        
        telemetry.addData("Status", "Initialized at TOP");
        telemetry.update();
        
        waitForStart();

        // ============================================
        // PHASE 0: THE INITIAL DROP (Top -> Bottom)
        // ============================================
        
        telemetry.addData("Status", "Initial Drop...");
        telemetry.update();

        // 1. Configure for Downward Move
        myMotor0.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, downPIDF40);
        myMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, downPIDF60);
        myMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, downPIDF60);
        myMotor3.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, downPIDF40);

        // 2. Set Target to Bottom (Positive direction relative to top start)
        myMotor0.setTargetPosition(INITIAL_DROP_DISTANCE);
        myMotor1.setTargetPosition((int)(INITIAL_DROP_DISTANCE * ratio_offset));
        myMotor2.setTargetPosition((int)(INITIAL_DROP_DISTANCE * ratio_offset));
        myMotor3.setTargetPosition(INITIAL_DROP_DISTANCE);
        
        myMotor0.setVelocity(LOWER_VEL);
        myMotor1.setVelocity((int) (LOWER_VEL * ratio_offset));
        myMotor2.setVelocity((int) (LOWER_VEL * ratio_offset));
        myMotor3.setVelocity(LOWER_VEL);

        // 3. Wait for it to get close to the water
        runtime.reset();
        while (opModeIsActive() && (myMotor0.isBusy() || myMotor1.isBusy()) && runtime.seconds() < 2.5) {
             telemetry.addData("Status", "Dropping to water...");
             telemetry.update();
        }

        // 4. Switch to Dunk Mode (Power)
        myMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 5. Push firmly into the B-Stop
        double dunkPower = 0.4;
        myMotor0.setPower(dunkPower);
        myMotor1.setPower(dunkPower);
        myMotor2.setPower(dunkPower);
        myMotor3.setPower(dunkPower);
        
        sleep(1000); // Wait for settle

        // 6. RESET ZERO (Critical Step)
        // Now "0" is at the bottom, just like the original code expects.
        myMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        sleep(200);

        // ============================================
        // NORMAL LOOP STARTS HERE (Identical to your working code)
        // ============================================
        for(int i = 0; i < 6; i++) {
            
            telemetry.addData("Loop Count", i + 1);
            telemetry.update();

            // ------------------------------------
            // PHASE 1: GO UP
            // ------------------------------------
            myMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            myMotor0.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, liftPIDF40);
            myMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, liftPIDF60);
            myMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, liftPIDF60);
            myMotor3.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, liftPIDF40);

            // Set Targets
            myMotor0.setTargetPosition(UP_POSITION);
            myMotor1.setTargetPosition((int)(UP_POSITION * ratio_offset));
            myMotor2.setTargetPosition((int)(UP_POSITION * ratio_offset));
            myMotor3.setTargetPosition(UP_POSITION);
            
            // Set Velocity
            myMotor0.setVelocity(LIFT_VEL);
            myMotor1.setVelocity((int) (LIFT_VEL * ratio_offset));
            myMotor2.setVelocity((int) (LIFT_VEL * ratio_offset));
            myMotor3.setVelocity(LIFT_VEL);
            
            runtime.reset();
            // Wait for completion (with Manual Soft-Stop Logic)
            while (opModeIsActive() && (myMotor0.isBusy() || myMotor1.isBusy()) && runtime.seconds() < 10.0) {
                if(Math.abs(myMotor0.getCurrentPosition() - UP_POSITION) < 100) {
                    LIFT_VEL = 150;
                } else {
                    LIFT_VEL = 600;
                }
                
                myMotor0.setVelocity(LIFT_VEL);
                myMotor1.setVelocity((int) (LIFT_VEL * ratio_offset));
                myMotor2.setVelocity((int) (LIFT_VEL * ratio_offset));
                myMotor3.setVelocity(LIFT_VEL);
                    
                telemetry.addData("Loop", i + 1);
                telemetry.addData("Status", "Moving UP");
                telemetry.update();
            }
            
            // Active Hold
            myMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            myMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            myMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
            double upHoldPower = -0.04;
            myMotor0.setPower(upHoldPower);
            myMotor1.setPower(upHoldPower);
            myMotor2.setPower(upHoldPower);
            myMotor3.setPower(upHoldPower);
            
            sleep(1200); 

            // ------------------------------------
            // PHASE 2: GO DOWN
            // ------------------------------------
            myMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            myMotor0.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, downPIDF40);
            myMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, downPIDF60);
            myMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, downPIDF60);
            myMotor3.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, downPIDF40);
            
            // Set Targets
            myMotor0.setTargetPosition(DOWN_POSITION);
            myMotor1.setTargetPosition((int)(DOWN_POSITION * ratio_offset));
            myMotor2.setTargetPosition((int)(DOWN_POSITION * ratio_offset));
            myMotor3.setTargetPosition(DOWN_POSITION);
            
            // Set Velocity
            myMotor0.setVelocity(LOWER_VEL);
            myMotor1.setVelocity((int) (LOWER_VEL * ratio_offset));
            myMotor2.setVelocity((int) (LOWER_VEL * ratio_offset));
            myMotor3.setVelocity(LOWER_VEL);
            
            runtime.reset();
            // Wait for completion
            while (opModeIsActive() && (myMotor0.isBusy() || myMotor1.isBusy()) && runtime.seconds() < 1.5) {
                telemetry.addData("Loop", i + 1);
                telemetry.addData("Status", "Moving DOWN");
                telemetry.update();
            }

            // Dunk
            myMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            myMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            myMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Dunk Power
            myMotor0.setPower(0.4);
            myMotor1.setPower(0.4);
            myMotor2.setPower(0.4);
            myMotor3.setPower(0.4);
            
            sleep(1500);
            
            // Reset Encoders
            myMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            myMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            myMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            myMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            
            // Adjust Target with SAFETY CLAMP
            UP_POSITION -= 15;
            UP_POSITION = Math.max(UP_POSITION, -650); // Prevents it from going too far
        }

        telemetry.addData("Status", "Done!");
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class DropState extends State {

    private Servo armServo;
    private Servo swingman;
    private DcMotor railMotor;
    private final int waitTime = 3;

    /**
     * This is the default State constructor.
     *
     * @param stateMachine The statemachine sequence to which the state belongs.
     */
    public DropState(StateMachine stateMachine) {
        super(stateMachine);
    }

    @Override
    void start() {

    }

    @Override
    void update() {
        if (!(elapsedTime.seconds() < 4)) {
            startNextState();
        } else {
            this.swingman.setPosition(1);
            this.armServo.setPosition(0);
        }
    }

    @Override
    void stop() {
        this.railMotor.setPower(0);
    }

    @Override
    void initialize() {
        this.armServo = this.hardwareMap.get(Servo.class, "cs");
        this.swingman = this.hardwareMap.get(Servo.class, "ts");
        this.railMotor = this.hardwareMap.get(DcMotor.class, "xr");
    }
}

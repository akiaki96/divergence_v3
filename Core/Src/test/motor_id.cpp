#include "test/motor_id.hpp"


onenter(lamp_001sec, 
    motorDriver.setDuty(0.f, 0.f);

    logger.initLoggedVal();
    logger.add(
        "left_encoder_velocity",
        etl::delegate<float()>::create<Encoder, &Encoder::velocity>(encoderLeft)
    );
    logger.add(
        "right_encoder_velocity",
        etl::delegate<float()>::create<Encoder, &Encoder::velocity>(encoderRight)
    );
    logger.add(
        "battery",
        etl::delegate<float()>::create<Battery, &Battery::filtered>(adcValue.batt)
    );
    logger.add(
        "Left Duty",
        etl::delegate<float()>::create<Motor, &Motor::getDuty>(motorLeft)
    );
    logger.add(
        "Right Duty",
        etl::delegate<float()>::create<Motor, &Motor::getDuty>(motorRight)
    );

    ledBar16.set(0xFFFF);
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    HAL_Delay(500);

    motorDriver.state = MotorDriverState::lampDuty;
    motorDriver.setLampGrad(0.f);

    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);
    motorDriver.setLampGrad(0.01f);
    HAL_Delay(1000);
    logger.stop();
    motorDriver.setBreak();
    HAL_Delay(500);
    ledBar16.set(0xFFFF);
)
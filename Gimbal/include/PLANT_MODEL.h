#ifndef PLANT_MODEL_H
#define PLANT_MODEL_H

class PlantModel {

private:
    float rollAngle;
    float pitchAngle;

    float rollVelocity;
    float pitchVelocity;

public:
    PlantModel();

    void update(float rollControl, float pitchControl, float dt);

    float getRollAngle();
    float getPitchAngle();

    float getRollVelocity();
    float getPitchVelocity();
};

#endif
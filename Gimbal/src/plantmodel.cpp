#include "PLANT_MODEL.h"

PlantModel::PlantModel()
    : rollAngle(10.0),
      pitchAngle(-5.0),
      rollVelocity(0.0),
      pitchVelocity(0.0) {}

void PlantModel::update(float rollControl, float pitchControl, float dt) {

    // Simulate angular acceleration from control effort
    rollVelocity += rollControl * dt;
    pitchVelocity += pitchControl * dt;

    // Simple damping term to prevent infinite oscillation
    rollVelocity *= 0.98;
    pitchVelocity *= 0.98;

    // Update angles
    rollAngle += rollVelocity * dt;
    pitchAngle += pitchVelocity * dt;
}

float PlantModel::getRollAngle() {
    return rollAngle;
}

float PlantModel::getPitchAngle() {
    return pitchAngle;
}

float PlantModel::getRollVelocity() {
    return rollVelocity;
}

float PlantModel::getPitchVelocity() {
    return pitchVelocity;
}
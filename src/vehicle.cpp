#include "../include/vehicle.h"

#include <cmath>

Vehicle::Vehicle() {
    m_rotatingMass = nullptr;
    m_mass = 0;
    m_dragCoefficient = 0;
    m_crossSectionArea = 0;
    m_diffRatio = 0;
    m_tireRadius = 0;
    m_travelledDistance = 0;
    m_rollingResistance = 0;

    m_brakePressure = 0.0;
    m_maxBrakeForce = 0.0;
}

Vehicle::~Vehicle() {
    /* void */
}

void Vehicle::initialize(const Parameters& params) {
    m_mass = params.mass;
    m_dragCoefficient = params.dragCoefficient;
    m_crossSectionArea = params.crossSectionArea;
    m_diffRatio = params.diffRatio;
    m_tireRadius = params.tireRadius;
    m_rollingResistance = params.rollingResistance;

    if (params.maxBrakeForce > 0.0) {
        m_maxBrakeForce = params.maxBrakeForce;
    }
    else {
        // Default brake force equivalent to approximately 1 g.
        m_maxBrakeForce = m_mass * 9.81;
    }
    m_brakePressure = 0.0;
}

void Vehicle::setBrakePressure(double pressure) {
    if (pressure < 0.0) {
        pressure = 0.0;
    }
    else if (pressure > 1.0) {
        pressure = 1.0;
    }

    m_brakePressure = pressure;
}

void Vehicle::update(double dt) {
    m_travelledDistance += getSpeed() * dt;
}

void Vehicle::addToSystem(atg_scs::RigidBodySystem* system, atg_scs::RigidBody* rotatingMass) {
    m_rotatingMass = rotatingMass;
}

double Vehicle::getSpeed() const {
    const double E_r = 0.5 * m_rotatingMass->I * m_rotatingMass->v_theta * m_rotatingMass->v_theta;
    const double vehicleSpeed = std::sqrt(2 * E_r / m_mass);

    return vehicleSpeed;

    // E_r = 0.5 * I * v_theta^2
    // E_k = 0.5 * m * v^2
}

double Vehicle::linearForceToVirtualTorque(double force) const {
    const double rotationToKineticRatio =
        std::sqrt(m_rotatingMass->I / m_mass);
    return rotationToKineticRatio * force;
}

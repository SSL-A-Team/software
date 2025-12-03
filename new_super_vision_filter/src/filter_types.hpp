#ifndef FILTER_TYPES_HPP_ 
#define FILTER_TYPES_HPP_ 

#include "kalman/Types.hpp"
#include "kalman/LinearizedMeasurementModel.hpp"

/*
    Position measurement for robot or balls

    Structure is {
        x_pos
        y_pos
    }

    Measurements are in m
*/
class PosMeasurement : public Kalman::Vector<double, 2>
{
public:
    KALMAN_VECTOR(PosMeasurement, double, 2)

    double x() const {
        return (*this)[0];
    }
    
    double y() const {
        return (*this)[1];
    }
};

/*
    4D state vector for robot or ball position
    
    State vector structure is    
        { x_pos,
        y_pos,
        x_vel,
        y_vel }
    
    Measurements are in m
*/
class PosState : public Kalman::Vector<double, 4>
{
public:
    KALMAN_VECTOR(State, double, 4)

    static constexpr size_t PX = 0;
    static constexpr size_t PY = 1;
    static constexpr size_t VX = 2;
    static constexpr size_t VY = 3;
};

/*
    Measurement model for robots/balls' X and Y pos + velocity from
    position measurements

    Curently assumes no measurement noise
*/
class PosMeasurementModel
    : public Kalman::LinearizedMeasurementModel<PosState, PosMeasurement, StandardBase>
{
public:
    // h(x) = predicted measurement
    PosMeasurement h(const PosState& x) const 
    {
        PosMeasurement z;
        z[0] = x[0]; // px
        z[1] = x[1]; // py
        return z;
    }

    // Jacobian H = ∂h/∂x
protected:
    void updateJacobians(const PosState& x)
    {
        this->H.setZero();

        this->H(0, 0) = 1; // dz_px / d_px
        this->H(1, 1) = 1; // dz_py / d_py
    }
};

/*
    Angular position measurement (in rad)
*/
class AngleMeasurement : public KalmanVector<double, 1>
{
public:
    KALMAN_VECTOR(AngleMeasurement, double, 1)
};

/*
    Angular state (position and velocity)

    State vector format is
    {
        w_pos
        w_vel
    }
*/
class AngleState : public Kalman::Vector<double, 2>
{
public:
    KALMAN_VECTOR(State, double, 2)

    static constexpr size_t PW = 0;
    static constexpr size_t VW = 1;
};

/*
    Linearized angle measurement model

    Currently assumes no measurement noise
*/
class AngleMeasurementModel
    : public Kalman::LinearizedMeasurementModel<AngleState, AngleMeasurement, StandardBase>
{
public:
    // h(x) = predicted measurement
    AngleMeasurement h(const AngleState& x) const override
    {
        AngleMeasurement z;
        z[0] = x[0]; // pw
        return z;
    }

    // Jacobian H = ∂h/∂x
    void updateJacobians(const StateType& x) override
    {
        this->H.setZero();
        this->H(0, 0) = 1; // dz_px / d_px
    }
};

#endif // FILTER_TYPES_HPP
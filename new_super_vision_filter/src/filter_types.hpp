#ifndef FILTER_TYPES_HPP_ 
#define FILTER_TYPES_HPP_ 

#include <chrono>

#include "kalman/Types.hpp"
#include "kalman/LinearizedMeasurementModel.hpp"
#include "kalman/LinearizedSystemModel.hpp"

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
    
    static constexpr size_t X = 0;
    static constexpr size_t Y = 1;

    double x() const {
        return (*this)[X];
    }
    
    double y() const {
        return (*this)[Y];
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
    KALMAN_VECTOR(PosState, double, 4)

    static constexpr size_t PX = 0;
    static constexpr size_t PY = 1;
    static constexpr size_t VX = 2;
    static constexpr size_t VY = 3;

    const double px(){return (*this)[PX]; }
    const double py(){return (*this)[PY]; }
    const double vx(){return (*this)[VX]; }
    const double vy(){return (*this)[VY]; }

};

/*
    Measurement model for robots/balls' X and Y pos + velocity from
    position measurements

    Curently assumes no measurement noise
*/
class PosMeasurementModel
    : public Kalman::LinearizedMeasurementModel<PosState, PosMeasurement, Kalman::StandardBase>
{
public:
    // h(x) = predicted measurement
    PosMeasurement h(const PosState& x) const 
    {
        PosMeasurement z;
        z[0] = x(x.PX); // px
        z[1] = x(x.PY); // py
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
    System (measurement to state) transition model for X/Y position
    that assumes a constant velocity and no control inputs.

*/
class PosSystemModel : public Kalman::LinearizedSystemModel<PosState>
{
    private:
        const float ms_to_s = 1e4;
        std::chrono::time_point<std::chrono::system_clock> last_update;

    public:
        // For now, we don't add any control inputs to the vision system
        // TODO (Christian): We might need this for the robots, even if we
        // don't use it for the ball
        using Control = Kalman::Vector<double, 0>;

        /*
            The f() function applies what would be the A (state transition)
            matrix but lazily skips doing a matmul.

            We assume the velocity stays constant and add v * dt to the
            current position.

            pos_t = pos_{t-1} + vel_{t-1} * dt
            vel_t = vel_{t-1}
        */
        PosState f(const PosState& x, const Control& /*u*/) const override
        {
            PosState x_updated{};
            auto now = std::chrono::system_clock::now();
            std::chrono::duration<float, std::milli> dt = now - last_update; 
            
            // B/c dt is in ms, we need to convert to s, since
            // our velocities are all in m/s
            x_updated(x.PX) = x(x.PX) + x(x.VX) * dt.count() * ms_to_s;
            // We assume constant velocity in the system model
            x_updated(x.VX) = x(x.VX);
            x_updated(x.PY) = x(x.PY) + x(x.VY) * dt.count() * ms_to_s;
            x_updated(x.VY) = x(x.VY);

            last_update = now;
            return x_updated;
        }

};

/*
    Angular position measurement (in rad)
*/
class AngleMeasurement : public Kalman::Vector<double, 1>
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
    KALMAN_VECTOR(AngleState, double, 2)

    static constexpr size_t PW = 0;
    static constexpr size_t VW = 1;
    
    const double pw(){return (*this)[PW]; }
    const double vw(){return (*this)[VW]; }
};

/*
    Linearized angle measurement model

    Currently assumes no measurement noise
*/
class AngleMeasurementModel
    : public Kalman::LinearizedMeasurementModel<AngleState, AngleMeasurement, Kalman::StandardBase>
{
public:
    // h(x) = predicted measurement
    AngleMeasurement h(const AngleState& x) const override
    {
        AngleMeasurement z{};
        z[0] = x[0]; // pw
        return z;
    }

    // Jacobian H = ∂h/∂x
    void updateJacobians(const AngleState& x) override
    {
        this->H.setZero();
        this->H(0, 0) = 1; // dz_px / d_px
    }
};

class AngleSystemModel : public Kalman::LinearizedSystemModel<AngleState>
{
    private:
        const float ms_to_s = 1e4;
        std::chrono::time_point<std::chrono::system_clock> last_update;

    public:
        /*
            This is the system's state transition function (applies
            the A matrix)

            We assume the velocity stays constant and add v * dt to the
            current position.

            pos_t = pos_{t-1} + vel_{t-1} * dt
            vel_t = vel_{t-1}
        */
        AngleState f(const AngleState& x, const Control& ) const {
            AngleState x_updated{};

            const auto now = std::chrono::system_clock::now();
            std::chrono::duration<float, std::milli> dt = now - last_update;

            x_updated(x.PW) = x(x.PW) + x(x.VW) * dt.count() * ms_to_s;

            return x_updated;
        }
};

#endif // FILTER_TYPES_HPP
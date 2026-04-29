# Core NavKit Architecture Design Notes

Use C++ header config files to define compile time types, aliases, and variables
```
struct Config {
    using Scalar_t = float;
    static constexpr size_t ImuBufferSize = 256;
};
```

## State Vector Definition
All compile time state definitions - static polymorphism design principle. Simple `Segment` fundamental utility class template.
```
template<int I, int SZ>
struct Segment {
    static constexpr int i = I;
    static constexpr int sz = SZ;
};
```

Helper utility functions to grab sub-selections of matrices and vectors
```
// only useful for grabbing square sub-matrix on diagonal
template<typename TSeg, typename TMat>
auto block(const TMat& m) {
    return m.template block<TSeg::sz, TSeg::sz>(TSeg::i, TSeg::i);
}

template<typename TSeg, typename TVec>
auto segment(const TVec& v) {
    return v.template segment<TSeg::sz>(TSeg::i);
}

// use case
Eigen::Vector<Scalar_t, StateDef::Pos::sz> d_pos = segment<StateDef::Pos>(x);
```

Example standard StateDef
```
struct InsStateDef {
    // standard navigation states
    using Pos     = Segment<0, 3>;
    using Vel     = Segment<3, 3>;
    using Att     = Segment<6, 3>;

    // IMU error states, bias and scale factor
    using GyroB  = Segment<9, 3>;
    using GyroSf = Segment<12, 3>;
    using AccB   = Segment<15, 3>;
    using AccSf  = Segment<18, 3>;

    // size of the state vector
    static constexpr int N = 21;
};
```

Tightly coupled GNSS StateDef
```
struct GnssTcStateDef {
    // standard navigation states
    using Pos     = Segment<0, 3>;
    using Vel     = Segment<3, 3>;
    using Att     = Segment<6, 3>;

    // IMU error states - bias and scale factor
    using GyroB  = Segment<9, 3>;
    using GyroSf = Segment<12, 3>;
    using AccB   = Segment<15, 3>;
    using AccSf  = Segment<18, 3>;

    // GNSS receiver clock error states - bias and drift
    using ClkB  = Segment<21, 1>;
    using ClkD  = Segment<22, 1>;

    // size of the state vector
    static constexpr int N = 23;
};
```

Create a header that defines useful aliases and avoid circular dependencies
```
// Inside State.h
template<typename StateDef>
using State = Eigen::Vector<Scalar_t, StateDef::N>;

template<typename StateDef>
using StateCov = Eigen::Matrix<Scalar_t, StateDef::N, StateDef::N>;
```

## Kalman Filter Concepts

GNSS measurement class template with "StateDef". All measurement classes should provide types and definitions for H (observation) and R (covariance) matrices - Static polymorphism.
```
#include "State.h"

template<typename StateDef>
class GnssPosModel {
public:
    static constexpr int M = 3;
    // or could do
    // static constexpr int M = StateDef::Pos::sz;
    using H_t = Eigen::Matrix<Scalar_t, M, StateDef::N>;
    using R_t = Eigen::Matrix<Scalar_t, M, M>;
    using O_t = Eigen::Vector<Scalar_t, M>;
    using K_t = Eigen::Matrix<Scalar_t, StateDef::N, M>;

    // returns const reference for performance when can be statically initialized
    static const H_t& compute_h()
    {
        // simple definition
        // H_t H = H_t::Zero();
        // H.block<M, M>(0, StateDef::Pos::i) = R_t::Identity();
        // return H;

        // static local initialization with lambda
        static H_t H = [] {
            H_t tmp = H_t::Zero();
            tmp.block<M, M>(0, StateDef::Pos::i) = 
                Eigen::Matrix<Scalar_t, M, M>::Identity();
            return tmp;
        }();
        return H;
    }

    // more complex observation Jacobian might involve state vector in the future
    // H_t compute_h(const State& x);

    /*
    static R_t compute_r(const Scalar_t sig)
    {
        // or do dynamic more complicated measurement covariance matrix calculation
        return sig * sig * R_t::Identity();
    }
    */

    static R_t compute_r()
    {
        // simple placeholder
        // TODO: figure out exactly how class instances and configurations tie into this static function
        return R_t::Identity();
    }

    // observation function of whole state
    static O_t obs(const State& x)
    {
        return segment<StateDef::Pos>(x);
        // or
        // return x.segment<StateDef::Pos::sz>(StateDef::Pos::i);
        // or whatever more complex non-linear observation logic would happen here
    }
};
```

Creating sensors
```
GnssPosModel<InsStateDef>
BaroModel<InsStateDef>
MagModel<InsStateDef>
```

Main Kalman update funciton template that uses static polymorphism. Could use a simple container to manage variously configured sensor models, such as `std::tuple<>`.
```
using Sensors = std::tuple<
    GnssPosModel<StateDef>,
    BaroModel<StateDef>
>;
```

```
#include "State.h"

template<typename StateDef>
class KalmanFilter {
public:
    using State_t = State<StateDef>;
    using P_t = StateCov<StateDef>;

    // statically stored identity matrix for re-use
    static const P_t& I()
    {
        static P_t I = P_t::Identity();
        return I;
    }

    // could make the TSensor type explicitly require a template parameter `StateDef`
    // template <typename> TSensor>
    // and below you'd see `TSensor<StateDef>`
    template<typename TSensor>
    void covariance_update(
        const P_t& P_i,
        const typename TSensor::H_t& H,
        const typename TSensor::R_t& R,
        const typename TSensor::O_t& innovation,
        typename TSensor::R_t& S,
        typename TSensor::K_t& K,
        State_t& dx,
        P_t& P_f)
    {
        // innovation covariance in observation space
        S = H * P_i * H.transpose() + R;

        // "solve()" instead of inverse for numerical stability
        K = P_i * H.transpose() * S.ldlt().solve(TSensor::R_t::Identity());

        // Kalman observation correction
        dx = K * innovation;

        // Joseph form posteriori covariance for numerical stability
        P_f = (I() - K * H) * P_i * (I() - K * H).transpose() + K * R * K.transpose();
    }

    void inject_error(const State_t& dx)
    {
        // do more complicated logic to handle non linear addition, like for attitude
        m_x += dx;
    }

    // "x" is whole state prior to observation (priori)
    // "z" is measurement observation
    template<typename TSensor>
    void observation_update(const typename TSensor::O_t& z)
    {
        // calculate innovation given non-linear measurement observation model
        typename TSensor::O_t innov = z - TSensor::obs(m_x);

        // update covariance given observation
        typename TSensor::R_t S;
        typename TSensor::K_t K;
        State_t dx;
        covariance_update<TSensor>(
            m_P_i,
            TSensor::compute_h(),
            TSensor::compute_r(),
            innov,
            S,
            K,
            dx,
            m_P_f
        );

        // inject error state correction to whole state
        inject_error(dx);
    }

    // function template to process sensor
    // TODO: define sensor class
    template<typename TSensor>
    void process_sensor(TSensor& sensor)
    {
        while (sensor.has_measurement()) {
            // sensor will provide data that has measurement
            auto meas = sensor.pop();
            observation_update(meas.z);
        }
    }

private:
    // whole state member variable
    State_t m_x;

    // member priori (i) and posteriori (f) covariance matrices
    P_t m_P_i;
    P_t m_P_f;
}
```

## Navigator
The navigator will own the `KalmanFilter` and all `Sensor`s, and will orchestrate all main logic.

Starting with an example of some concrete `Sensor` types, and how we can use those to create an aliased `Sensors` tuple that's configured at compile time.
```
// define state layout
using StateDef = InsStateDef;

// define sensors to be used
using Sensors = std::tuple<
    GnssPosSensor<StateDef>,
    BaroSensor<StateDef>
>;

// define navigator type
using Nav = Navigator<StateDef, Sensors>;

// instantiate navigator
Nav nav;
```

Navigator class:

```
template<typename TStateDef, typename TSensors>
class Navigator
{
public:
    using KF_t = KalmanFilter<TStateDef>;

    void process_measurements()
    {
        process_all_sensors();
    }

private:
    /*
    Compile time generic sensor processing of an `std::tuple<>` of `Sensor`s. Uses `std::apply()` (C++17) to effectively iterate through the tuple container at compile time.
    */
    void process_all_sensors()
    {
        std::apply(
            [this](auto&... sensor)
            {
                (m_filter.process_sensor(sensor), ...);
            },
            m_sensors
        );
    }

private:
    KF_t m_filter{};
    TSensors m_sensors{};
};
```

## Errors and Return Types
Here are some ideas for some basic return types.

Basic status return type class.
```
enum class Status {
    OK,
    NoData,
    Invalid,
    Overflow
};

// basic use case
Status read(ImuData& out);
```

Slightly more complex status return type option.
```
template<typename T>
struct Result {
    T value;
    Status status;
};

// basic use case
Result<ImuData> read(InputData& in)
{
    ImuData out{};
    if (in.is_empty()) {
        return Result<ImuData>{out, Status::NoData};
    }

    // do processing
    return Result<ImuData>{out, Status::OK};
}
```

Error types more specific to expected errors during filtering math procesing.
```
enum class NavStatus {
    OK,
    NumericalIssue,
    CovarianceNotPSD,
    BadMeasurement
};
```

`NavStatus` return type use case.
```
NavStatus updateGnss(const GnssMeas& z) {
    if (!isValid(z)) {
        return NavStatus::BadMeasurement;
    }

    // do update

    if (!P.allFinite()) {
        return NavStatus::NumericalIssue;
    }

    return NavStatus::OK;
}
```

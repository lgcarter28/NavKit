# Core NavKit Architecture Design Notes

Use C++ header config files to define compile time types, aliases, and variables
```
struct Config {
    using Scalar_t = float;
    static constexpr size_t ImuBufferSize = 256;
};
```

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
        Eigen::Matrix<Scalar_t, StateDef::N, TSensor::M>& K,
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
        Eigen::Matrix<Scalar_t, StateDef::N, TSensor::M> K;
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

private:
    // whole state member variable
    State_t m_x;

    // member priori (i) and posteriori (f) covariance matrices
    P_t m_P_i;
    P_t m_P_f;
}
```

Need to think through navigator and how it will configure and store its `KalmanFitler`, whatever `Sensor`s, and who owns what. What will require class instances, and what will be built into types?

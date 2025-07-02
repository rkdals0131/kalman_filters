# Kalman Filters
This is a C++ implementation of the Kalman filter and its extensions: the Extended Kalman Filter (EKF) and the Unscented Kalman Filter (UKF).

## Features
- Header-only design with minimal dependencies (only Eigen3)
- Abstract `SystemModel` base class for easy customization
- Support for both EKF and UKF
- Clean, simple API
- Example implementations included

## Installation

### System-wide Installation
```bash
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make
sudo make install
```

### Using in Other Projects

#### Method 1: With CMake Package (Recommended)
```cmake
find_package(kalman_filters REQUIRED)
target_link_libraries(your_target kalman_filters::kalman_filters)
```

#### Method 2: Direct Library Linking
```cmake
# The actual library file is libkalman_filters_lib.so or .a
find_library(KALMAN_FILTERS_LIB kalman_filters_lib)
target_link_libraries(your_target ${KALMAN_FILTERS_LIB})
target_include_directories(your_target PRIVATE /usr/local/include)
```

#### For CALICO Package
In calico's CMakeLists.txt, add:
```cmake
# Find the installed library (libkalman_filters_lib.so/a)
find_library(KALMAN_FILTERS_LIB kalman_filters_lib REQUIRED)

# Link to the library (calico is the main library target)
target_link_libraries(calico 
  ${KALMAN_FILTERS_LIB}
  # ... other dependencies
)
```

## Usage Guide

### Step 1: Include Headers
```cpp
#include <ukf.hpp>        // For UKF
#include <ekf.hpp>        // For EKF
#include <system_model.hpp>  // Base class
```

### Step 2: Create Your System Model
```cpp
class ConeTrackingModel : public SystemModel {
public:
    ConeTrackingModel(const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q, double dt)
        : SystemModel(Eigen::Vector4d::Zero(), R, Q, dt) {}
    
    // For constant velocity model: [x, y, vx, vy]
    Eigen::VectorXd DynamicsModel(const Eigen::VectorXd& x, const Eigen::VectorXd& u) override {
        Eigen::VectorXd x_new = x;
        x_new(0) += dt * x(2);  // x += vx * dt
        x_new(1) += dt * x(3);  // y += vy * dt
        return x_new;
    }
    
    // Observe position only: [x, y]
    Eigen::VectorXd ObservationModel(const Eigen::VectorXd& x) override {
        return x.head(2);  // Return [x, y]
    }
    
    // Jacobians for EKF (optional for UKF)
    Eigen::MatrixXd JacobDynamicsModel(const Eigen::VectorXd& x, const Eigen::VectorXd& u) override {
        Eigen::MatrixXd F = Eigen::Matrix4d::Identity();
        F(0, 2) = dt;
        F(1, 3) = dt;
        return F;
    }
    
    Eigen::MatrixXd JacobObservationModel(const Eigen::VectorXd& x) override {
        Eigen::MatrixXd H(2, 4);
        H << 1, 0, 0, 0,
             0, 1, 0, 0;
        return H;
    }
};
```

### Step 3: Use the Filter
```cpp
// Setup noise matrices
Eigen::MatrixXd R = Eigen::Matrix2d::Identity() * 0.1;  // Measurement noise
Eigen::MatrixXd Q = Eigen::Matrix4d::Identity() * 0.01; // Process noise
Eigen::MatrixXd P = Eigen::Matrix4d::Identity();        // Initial covariance

// Create model and filter
auto model = std::make_shared<ConeTrackingModel>(R, Q, 0.033); // 30Hz
UnscentedKF ukf(model, P);

// Initialize with initial state
Eigen::Vector4d x0(1.0, 2.0, 0.0, 0.0);  // [x=1, y=2, vx=0, vy=0]
ukf.init(x0);

// Main loop
while (running) {
    // Predict step (u is control input, can be zero vector)
    Eigen::VectorXd u = Eigen::Vector2d::Zero();
    ukf.predict(u);
    
    // Update step with measurement
    Eigen::Vector2d measurement(1.1, 2.1);  // [x, y] from sensor
    ukf.update(measurement);
    
    // Get results
    Eigen::VectorXd state = ukf.get_state();
    Eigen::MatrixXd covariance = ukf.get_cov();
}
```

## Kalman Filter

## Extended Kalman Filter (EKF)

The extended Kalman filter is an extension of the Kalman filter for nonlinear systems.  
This technique linearizes a model at a working point using Taylor series expansion. 

```c++
#include <ekf.hpp>

// model  : system model (sub-class of SystemModel class)
// P      : initial covariance matrix
ExtendedKF ekf(model, P);

// x0: initial state
ekf.init(x0);

// u: control vector
ekf.predict(u);

// x: previous state vector
ekf.update(x);

// get estimated state and covariance matrix
Eigen::VectorXd x_est = ekf.get_state();
Eigen::MatrixXd P_est = ekf.get_cov();
```

![ekf_gif](doc/ekf.gif)

## Unscented Kalman Filter (UKF)

The extended Kalman filter requires differentiable models and gives poor performance in highly nonlinear systesm.  
The unscented Kalman filter uses a deterministic sampling technique known as the unscented transformation to calculate statistics around the mean. This technique does not require differentiability of models.

```c++
// it's almost the same with the ekf.
#include <ukf.hpp>

// model  : system model (sub-class of SystemModel class)
// P      : initial covariance matrix
// scale  : scaling parameter for adjust the effects of third-order momentum. [optional]
UnscentedKF ukf(model, P, scale);

// x0: initial state
ukf.init(x0);

// u: control vector
ukf.predict(u);

// x: previous state vector
ukf.update(x);

// get estimated state and covariance matrix
Eigen::VectorXd x_est = ukf.get_state();
Eigen::MatrixXd P_est = ukf.get_cov();
```

![ukf_gif](doc/ukf.gif)

## Creating a Custom System Model

To use the filters, you need to create a custom system model by inheriting from `SystemModel`:

```cpp
class MySystemModel : public SystemModel {
public:
    MySystemModel(const Eigen::VectorXd& x0, 
                  const Eigen::MatrixXd& R, 
                  const Eigen::MatrixXd& Q, 
                  double dt)
        : SystemModel(x0, R, Q, dt) {}

    // State transition function: x_k+1 = f(x_k, u_k)
    Eigen::VectorXd DynamicsModel(const Eigen::VectorXd& x, 
                                  const Eigen::VectorXd& u) override {
        // Implement your dynamics model
    }

    // Observation function: z_k = h(x_k)
    Eigen::VectorXd ObservationModel(const Eigen::VectorXd& x) override {
        // Implement your observation model
    }

    // Jacobian of dynamics (only for EKF)
    Eigen::MatrixXd JacobDynamicsModel(const Eigen::VectorXd& x, 
                                       const Eigen::VectorXd& u) override {
        // Implement Jacobian of dynamics
    }

    // Jacobian of observation (only for EKF)
    Eigen::MatrixXd JacobObservationModel(const Eigen::VectorXd& x) override {
        // Implement Jacobian of observation
    }
};
```
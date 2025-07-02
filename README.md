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

## Complete Examples

### Example 1: Constant Velocity Model (2D Tracking)

```cpp
#include <ukf.hpp>
#include <ekf.hpp>
#include <system_model.hpp>
#include <Eigen/Dense>

// Constant velocity model for tracking objects in 2D
class ConstantVelocityModel : public SystemModel {
public:
    ConstantVelocityModel(double dt, double process_noise = 0.1, double measurement_noise = 0.5)
        : SystemModel(Eigen::Vector4d::Zero(), 
                     Eigen::Matrix2d::Identity() * measurement_noise,  // R
                     Eigen::Matrix4d::Identity() * process_noise,      // Q
                     dt) {}
    
    // State: [x, y, vx, vy]
    Eigen::VectorXd DynamicsModel(const Eigen::VectorXd& x, const Eigen::VectorXd& u) override {
        Eigen::VectorXd x_next = x;
        x_next(0) += dt * x(2);  // x = x + vx*dt
        x_next(1) += dt * x(3);  // y = y + vy*dt
        // Velocities remain constant (constant velocity assumption)
        return x_next;
    }
    
    // Measure position only: [x, y]
    Eigen::VectorXd ObservationModel(const Eigen::VectorXd& x) override {
        return x.head(2);
    }
    
    // Jacobian for EKF
    Eigen::MatrixXd JacobDynamicsModel(const Eigen::VectorXd& x, const Eigen::VectorXd& u) override {
        Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
        F(0, 2) = dt;  // dx/dvx = dt
        F(1, 3) = dt;  // dy/dvy = dt
        return F;
    }
    
    Eigen::MatrixXd JacobObservationModel(const Eigen::VectorXd& x) override {
        Eigen::MatrixXd H(2, 4);
        H << 1, 0, 0, 0,  // Observe x
             0, 1, 0, 0;  // Observe y
        return H;
    }
};

// Usage example
void trackObject() {
    // Create model
    auto model = std::make_shared<ConstantVelocityModel>(0.1);  // 10Hz
    
    // Initial covariance
    Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
    P.block<2,2>(0,0) *= 1.0;  // Position uncertainty
    P.block<2,2>(2,2) *= 10.0; // Velocity uncertainty
    
    // Create filters
    UnscentedKF ukf(model, P);
    ExtendedKF ekf(model, P);
    
    // Initialize with a state
    Eigen::Vector4d x0(10.0, 5.0, 1.0, 0.5);  // Start at (10,5) with velocity (1,0.5)
    ukf.init(x0);
    ekf.init(x0);
    
    // Simulate tracking
    for (int i = 0; i < 100; ++i) {
        // Predict (no control input for constant velocity)
        Eigen::VectorXd u = Eigen::VectorXd::Zero(0);
        ukf.predict(u);
        ekf.predict(u);
        
        // Get measurement (simulated)
        Eigen::Vector2d z(10.0 + i*0.1 + 0.1*randn(), 
                         5.0 + i*0.05 + 0.1*randn());
        
        // Update
        ukf.update(z);
        ekf.update(z);
        
        // Get estimates
        auto ukf_state = ukf.get_state();
        auto ekf_state = ekf.get_state();
    }
}
```

### Example 2: IMU-based Motion Model

```cpp
// Motion model using IMU (accelerometer + gyroscope)
class IMUMotionModel : public SystemModel {
private:
    Eigen::Matrix3d R_imu_to_world_;  // IMU to world rotation
    
public:
    // State: [x, y, z, vx, vy, vz, roll, pitch, yaw]
    IMUMotionModel(double dt, const Eigen::Matrix3d& R_imu_to_world)
        : SystemModel(Eigen::VectorXd::Zero(9),
                     Eigen::Matrix3d::Identity() * 0.1,    // R (position measurement)
                     Eigen::Matrix<double,9,9>::Identity() * 0.01,  // Q
                     dt),
          R_imu_to_world_(R_imu_to_world) {
        // Adjust process noise for different states
        Q.block<3,3>(0,0) *= 0.1;   // Position noise
        Q.block<3,3>(3,3) *= 0.5;   // Velocity noise  
        Q.block<3,3>(6,6) *= 0.01;  // Orientation noise
    }
    
    // u = [ax, ay, az, wx, wy, wz] (accelerations and angular velocities from IMU)
    Eigen::VectorXd DynamicsModel(const Eigen::VectorXd& x, const Eigen::VectorXd& u) override {
        Eigen::VectorXd x_next = x;
        
        // Extract states
        Eigen::Vector3d pos = x.segment<3>(0);
        Eigen::Vector3d vel = x.segment<3>(3);
        double roll = x(6), pitch = x(7), yaw = x(8);
        
        // Extract IMU measurements
        Eigen::Vector3d accel_imu = u.head<3>();
        Eigen::Vector3d omega_imu = u.tail<3>();
        
        // Convert acceleration to world frame
        Eigen::Matrix3d R_body_to_world = eulerToRotation(roll, pitch, yaw);
        Eigen::Vector3d accel_world = R_body_to_world * R_imu_to_world_ * accel_imu;
        accel_world(2) -= 9.81;  // Remove gravity
        
        // Update position and velocity
        x_next.segment<3>(0) = pos + vel * dt + 0.5 * accel_world * dt * dt;
        x_next.segment<3>(3) = vel + accel_world * dt;
        
        // Update orientation
        Eigen::Vector3d omega_world = R_body_to_world * R_imu_to_world_ * omega_imu;
        x_next(6) += omega_world(0) * dt;  // roll rate
        x_next(7) += omega_world(1) * dt;  // pitch rate
        x_next(8) += omega_world(2) * dt;  // yaw rate
        
        return x_next;
    }
    
    // Observe position only
    Eigen::VectorXd ObservationModel(const Eigen::VectorXd& x) override {
        return x.head<3>();  // [x, y, z]
    }
    
    // Helper function
    Eigen::Matrix3d eulerToRotation(double roll, double pitch, double yaw) {
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
        return (yawAngle * pitchAngle * rollAngle).matrix();
    }
    
    // Jacobians for EKF (simplified - only key terms)
    Eigen::MatrixXd JacobDynamicsModel(const Eigen::VectorXd& x, const Eigen::VectorXd& u) override {
        Eigen::Matrix<double,9,9> F = Eigen::Matrix<double,9,9>::Identity();
        
        // Position depends on velocity
        F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
        
        // Velocity depends on orientation (through rotation of acceleration)
        // This is simplified - full implementation would compute proper derivatives
        
        return F;
    }
    
    Eigen::MatrixXd JacobObservationModel(const Eigen::VectorXd& x) override {
        Eigen::Matrix<double,3,9> H = Eigen::Matrix<double,3,9>::Zero();
        H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
        return H;
    }
};

// Usage with IMU data
void trackWithIMU() {
    // IMU mounting rotation (identity if aligned)
    Eigen::Matrix3d R_imu = Eigen::Matrix3d::Identity();
    
    auto model = std::make_shared<IMUMotionModel>(0.01, R_imu);  // 100Hz IMU
    
    Eigen::Matrix<double,9,9> P = Eigen::Matrix<double,9,9>::Identity();
    UnscentedKF ukf(model, P);
    
    // Initialize at origin
    Eigen::VectorXd x0(9);
    x0.setZero();
    ukf.init(x0);
    
    // Process IMU data
    while (true) {
        // Get IMU data (example values)
        Eigen::VectorXd imu_data(6);
        imu_data << 0.1, 0.0, 9.9,    // accelerometer (with gravity)
                   0.0, 0.0, 0.1;     // gyroscope
        
        ukf.predict(imu_data);
        
        // Occasionally get position fix (e.g., from GPS)
        if (hasPositionMeasurement()) {
            Eigen::Vector3d gps_pos(1.0, 2.0, 0.0);
            ukf.update(gps_pos);
        }
    }
}
```

### Example 3: Nonlinear System (Robot with Bearing Measurements)

```cpp
// Robot tracking with bearing-only measurements
class BearingOnlyModel : public SystemModel {
public:
    BearingOnlyModel(double dt)
        : SystemModel(Eigen::Vector3d::Zero(),  // x0: [x, y, theta]
                     Eigen::Matrix<double,1,1>::Identity() * 0.1,  // R (bearing noise)
                     Eigen::Matrix3d::Identity() * 0.01,           // Q
                     dt) {}
    
    // Unicycle model: robot with velocity and angular velocity control
    // State: [x, y, theta], Control: [v, omega]
    Eigen::VectorXd DynamicsModel(const Eigen::VectorXd& x, const Eigen::VectorXd& u) override {
        double v = u(0);      // linear velocity
        double omega = u(1);  // angular velocity
        double theta = x(2);
        
        Eigen::Vector3d x_next;
        x_next(0) = x(0) + v * cos(theta) * dt;
        x_next(1) = x(1) + v * sin(theta) * dt;
        x_next(2) = x(2) + omega * dt;
        
        return x_next;
    }
    
    // Measure bearing angle to landmark at origin
    Eigen::VectorXd ObservationModel(const Eigen::VectorXd& x) override {
        Eigen::VectorXd z(1);
        z(0) = atan2(-x(1), -x(0));  // Bearing to origin
        return z;
    }
    
    // EKF Jacobians
    Eigen::MatrixXd JacobDynamicsModel(const Eigen::VectorXd& x, const Eigen::VectorXd& u) override {
        double v = u(0);
        double theta = x(2);
        
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0, 2) = -v * sin(theta) * dt;
        F(1, 2) = v * cos(theta) * dt;
        
        return F;
    }
    
    Eigen::MatrixXd JacobObservationModel(const Eigen::VectorXd& x) override {
        double x_pos = x(0);
        double y_pos = x(1);
        double r_sq = x_pos*x_pos + y_pos*y_pos;
        
        Eigen::MatrixXd H(1, 3);
        H(0, 0) = y_pos / r_sq;
        H(0, 1) = -x_pos / r_sq;
        H(0, 2) = 0;
        
        return H;
    }
};
```
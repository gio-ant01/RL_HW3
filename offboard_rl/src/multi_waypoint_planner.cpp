#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <vector>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <offboard_rl/utils.h>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

/**
 * Multi-Waypoint Trajectory Planner
 * 
 * Features:
 * - Minimum 7 waypoints
 * - UAV does NOT stop at intermediate waypoints (v=0 only at start and end)
 * - C2 continuity: position, velocity, acceleration continuous between segments
 * - Quintic (5th order) polynomial for each segment
 */
class MultiWaypointPlanner : public rclcpp::Node
{
public:
    MultiWaypointPlanner() : Node("multi_waypoint_planner")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos, 
            std::bind(&MultiWaypointPlanner::vehicle_local_position_callback, this, std::placeholders::_1));
        
        attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos, 
            std::bind(&MultiWaypointPlanner::vehicle_attitude_callback, this, std::placeholders::_1));
        
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        timer_offboard_ = this->create_wall_timer(100ms, std::bind(&MultiWaypointPlanner::activate_offboard, this));
        timer_trajectory_publish_ = this->create_wall_timer(20ms, std::bind(&MultiWaypointPlanner::publish_trajectory_setpoint, this));

        keyboard_thread_ = std::thread(&MultiWaypointPlanner::keyboard_listener, this);
    }

    ~MultiWaypointPlanner()
    {
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
    }

private:
    // ROS2 components
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_subscription_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::TimerBase::SharedPtr timer_offboard_;
    rclcpp::TimerBase::SharedPtr timer_trajectory_publish_;
    std::thread keyboard_thread_;

    // State variables
    bool waypoints_received_{false};
    bool offboard_active_{false};
    bool trajectories_computed_{false};
    double offboard_counter_{0};
    
    VehicleLocalPosition current_position_{};
    VehicleAttitude current_attitude_{};

    // Trajectory data
    std::vector<Eigen::Vector4d> waypoints_;  // [x, y, z, yaw] for each waypoint
    std::vector<double> segment_times_;       // Duration of each segment
    std::vector<Eigen::Vector<double, 6>> segment_coeffs_;  // Polynomial coefficients for each segment
    
    double total_time_{0.0};      // Total trajectory time
    double current_time_{0.0};    // Current time in trajectory execution
    int current_segment_{0};      // Current segment being executed

    // CALLBACKS
    
    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        current_position_ = *msg;
    }

    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        current_attitude_ = *msg;
    }

    // KEYBOARD INPUT THREAD
    
    void keyboard_listener()
    {
        while (rclcpp::ok() && !waypoints_received_)
        {
            std::cout << "\n========================================" << std::endl;
            std::cout << "MULTI-WAYPOINT TRAJECTORY PLANNER" << std::endl;
            std::cout << "========================================" << std::endl;
            std::cout << "Enter waypoints (minimum 7 required)" << std::endl;
            std::cout << "Format: x y z yaw (meters, meters, meters, radians)" << std::endl;
            std::cout << "Type 'done' when finished entering waypoints" << std::endl;
            std::cout << "----------------------------------------\n" << std::endl;

            waypoints_.clear();
            
            // Add current position as first waypoint
            Eigen::Vector4d start_waypoint;
            start_waypoint(0) = current_position_.x;
            start_waypoint(1) = current_position_.y;
            start_waypoint(2) = current_position_.z;
            auto rpy = utilities::quatToRpy(Eigen::Vector4d(
                current_attitude_.q[0], current_attitude_.q[1], 
                current_attitude_.q[2], current_attitude_.q[3]));
            start_waypoint(3) = rpy[2];
            waypoints_.push_back(start_waypoint);
            
            std::cout << "Waypoint 0 (START): [" << start_waypoint(0) << ", " 
                      << start_waypoint(1) << ", " << start_waypoint(2) 
                      << ", " << start_waypoint(3) << "]" << std::endl;

            // Get waypoints from user
            int wp_count = 1;
            while (rclcpp::ok())
            {
                std::cout << "Waypoint " << wp_count << ": ";
                std::string line;
                std::getline(std::cin, line);
                
                if (line == "done") {
                    if (waypoints_.size() < 7) {
                        std::cout << "ERROR: Need at least 7 waypoints total (including start). "
                                  << "Current count: " << waypoints_.size() << std::endl;
                        continue;
                    }
                    break;
                }

                std::istringstream iss(line);
                Eigen::Vector4d wp;
                if (!(iss >> wp(0) >> wp(1) >> wp(2) >> wp(3))) {
                    std::cout << "Invalid input. Please enter 4 numeric values." << std::endl;
                    continue;
                }
                
                wp(2) = -wp(2);  // Convert to NED frame
                waypoints_.push_back(wp);
                std::cout << "  Added: [" << wp(0) << ", " << wp(1) << ", " 
                          << wp(2) << ", " << wp(3) << "]" << std::endl;
                wp_count++;
            }

            // Get segment timing
            std::cout << "\nEnter time per segment (seconds, same for all segments): ";
            double time_per_segment;
            std::cin >> time_per_segment;
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            
            segment_times_.clear();
            for (size_t i = 0; i < waypoints_.size() - 1; i++) {
                segment_times_.push_back(time_per_segment);
            }
            
            total_time_ = time_per_segment * (waypoints_.size() - 1);
            
            std::cout << "\n========================================" << std::endl;
            std::cout << "TRAJECTORY SUMMARY" << std::endl;
            std::cout << "========================================" << std::endl;
            std::cout << "Total waypoints: " << waypoints_.size() << std::endl;
            std::cout << "Segments: " << waypoints_.size() - 1 << std::endl;
            std::cout << "Total time: " << total_time_ << " seconds" << std::endl;
            std::cout << "========================================\n" << std::endl;

            waypoints_received_ = true;
        }
    }

    // OFFBOARD ACTIVATION
    
    void activate_offboard()
    {
        if (!waypoints_received_) return;

        if (offboard_counter_ == 10) {
            // Switch to Offboard mode
            VehicleCommand msg{};
            msg.param1 = 1;
            msg.param2 = 6;
            msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
            msg.target_system = 1;
            msg.target_component = 1;
            msg.source_system = 1;
            msg.source_component = 1;
            msg.from_external = true;
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            vehicle_command_publisher_->publish(msg);

            // Arm the vehicle
            msg.param1 = 1.0;
            msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            vehicle_command_publisher_->publish(msg);

            offboard_active_ = true;
            RCLCPP_INFO(this->get_logger(), "Offboard mode activated!");
        }

        // Publish offboard control mode heartbeat
        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);

        if (offboard_counter_ < 11) offboard_counter_++;
    }

    // TRAJECTORY COMPUTATION
    
    void compute_trajectories()
    {
        segment_coeffs_.clear();
        
        // For each segment between waypoints
        for (size_t i = 0; i < waypoints_.size() - 1; i++) {
            Eigen::Vector4d wp_start = waypoints_[i];
            Eigen::Vector4d wp_end = waypoints_[i + 1];
            double T = segment_times_[i];
            
            // Compute direction and distance
            Eigen::Vector4d e = wp_end - wp_start;
            e(3) = utilities::angleError(wp_end(3), wp_start(3));  // Handle angle wrapping
            double s_f = e.norm();
            
            // Boundary conditions for this segment
            Eigen::VectorXd b(6);
            Eigen::Matrix<double, 6, 6> A;
            
            if (i == 0) {
                // First segment: zero velocity and acceleration at start
                b << 0.0, 0.0, 0.0, s_f, 0.0, 0.0;
                
                A << 0,           0,           0,          0,    0, 1,   // s(0) = 0
                     0,           0,           0,          0,    1, 0,   // ṡ(0) = 0
                     0,           0,           0,          1,    0, 0,   // s̈(0) = 0
                     pow(T,5),    pow(T,4),    pow(T,3),   T*T,  T, 1,   // s(T) = s_f
                     5*pow(T,4),  4*pow(T,3),  3*T*T,      2*T,  1, 0,   // ṡ(T) = v_end
                     20*pow(T,3), 12*T*T,      6*T,        2,    0, 0;   // s̈(T) = a_end
            }
            else if (i == waypoints_.size() - 2) {
                // Last segment: zero velocity and acceleration at end
                b << 0.0, 0.0, 0.0, s_f, 0.0, 0.0;
                
                A << 0,           0,           0,          0,    0, 1,
                     0,           0,           0,          0,    1, 0,
                     0,           0,           0,          1,    0, 0,
                     pow(T,5),    pow(T,4),    pow(T,3),   T*T,  T, 1,
                     5*pow(T,4),  4*pow(T,3),  3*T*T,      2*T,  1, 0,
                     20*pow(T,3), 12*T*T,      6*T,        2,    0, 0;
            }
            else {
                // Intermediate segment: match velocity and acceleration from previous segment
                // Get end conditions from previous segment
                double T_prev = segment_times_[i-1];
                Eigen::Vector<double, 6> x_prev = segment_coeffs_[i-1];
                
                Eigen::Vector4d e_prev = waypoints_[i] - waypoints_[i-1];
                e_prev(3) = utilities::angleError(waypoints_[i](3), waypoints_[i-1](3));
                double s_f_prev = e_prev.norm();
                
                // Velocity at end of previous segment
                double v_start = (5.0 * x_prev(0) * pow(T_prev, 4.0)
                                + 4.0 * x_prev(1) * pow(T_prev, 3.0)
                                + 3.0 * x_prev(2) * pow(T_prev, 2.0)
                                + 2.0 * x_prev(3) * T_prev
                                + x_prev(4));
                
                // Acceleration at end of previous segment
                double a_start = (20.0 * x_prev(0) * pow(T_prev, 3.0)
                                + 12.0 * x_prev(1) * pow(T_prev, 2.0)
                                + 6.0  * x_prev(2) * T_prev
                                + 2.0  * x_prev(3));
                
                // Scale to current segment
                double v_scaled = v_start * s_f / s_f_prev;
                double a_scaled = a_start * s_f / s_f_prev;
                
                b << 0.0, v_scaled, a_scaled, s_f, v_scaled, a_scaled;
                
                A << 0,           0,           0,          0,    0, 1,
                     0,           0,           0,          0,    1, 0,
                     0,           0,           0,          2,    0, 0,
                     pow(T,5),    pow(T,4),    pow(T,3),   T*T,  T, 1,
                     5*pow(T,4),  4*pow(T,3),  3*T*T,      2*T,  1, 0,
                     20*pow(T,3), 12*T*T,      6*T,        2,    0, 0;
            }
            
            // Solve for coefficients
            Eigen::Vector<double, 6> coeffs = A.inverse() * b;
            segment_coeffs_.push_back(coeffs);
            
            RCLCPP_INFO(this->get_logger(), "Segment %zu computed: [%.2f,%.2f,%.2f,%.2f] -> [%.2f,%.2f,%.2f,%.2f]",
                        i, wp_start(0), wp_start(1), wp_start(2), wp_start(3),
                        wp_end(0), wp_end(1), wp_end(2), wp_end(3));
        }
        
        trajectories_computed_ = true;
        //RCLCPP_INFO(this->get_logger(), "All trajectories computed successfully!");
    }

    // TRAJECTORY EXECUTION
    
    void publish_trajectory_setpoint()
    {
        if (!waypoints_received_ || !offboard_active_) return;
        
        if (!trajectories_computed_) {
            compute_trajectories();
            return;
        }
        
        /* if (current_time_ > total_time_) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory completed!");
            return;
        } */
        
        // Find current segment
        double segment_start_time = 0.0;
        for (size_t i = 0; i < segment_times_.size(); i++) {
            if (current_time_ <= segment_start_time + segment_times_[i]) {
                current_segment_ = i;
                break;
            }
            segment_start_time += segment_times_[i];
        }
        
        // Time within current segment
        double t_segment = current_time_ - segment_start_time;
        
        // Compute trajectory setpoint
        TrajectorySetpoint msg = compute_setpoint_for_segment(current_segment_, t_segment);
        trajectory_setpoint_publisher_->publish(msg);
        
        // Update time
        double dt = 1.0 / 50.0;  // 20ms = 50Hz
        current_time_ += dt;
    }
    
    TrajectorySetpoint compute_setpoint_for_segment(int segment_idx, double t)
    {
        Eigen::Vector4d wp_start = waypoints_[segment_idx];
        Eigen::Vector4d wp_end = waypoints_[segment_idx + 1];
        Eigen::Vector<double, 6> x = segment_coeffs_[segment_idx];
        
        // Direction and distance
        Eigen::Vector4d e = wp_end - wp_start;
        e(3) = utilities::angleError(wp_end(3), wp_start(3));
        double s_f = e.norm();
        
        // Evaluate polynomial at time t
        double s = x(0) * pow(t, 5.0)
                 + x(1) * pow(t, 4.0)
                 + x(2) * pow(t, 3.0)
                 + x(3) * pow(t, 2.0)
                 + x(4) * t
                 + x(5);
        
        double s_d = 5.0 * x(0) * pow(t, 4.0)
                   + 4.0 * x(1) * pow(t, 3.0)
                   + 3.0 * x(2) * pow(t, 2.0)
                   + 2.0 * x(3) * t
                   + x(4);
        
        double s_dd = 20.0 * x(0) * pow(t, 3.0)
                    + 12.0 * x(1) * pow(t, 2.0)
                    + 6.0  * x(2) * t
                    + x(3);
        
        // Convert to position, velocity, acceleration
        Eigen::Vector4d pos = wp_start + s * e / s_f;
        Eigen::Vector4d vel = s_d * e / s_f;
        Eigen::Vector4d acc = s_dd * e / s_f;
        
        // Create message
        TrajectorySetpoint msg{};
        msg.position = {static_cast<float>(pos(0)), 
                       static_cast<float>(pos(1)), 
                       static_cast<float>(pos(2))};
        msg.velocity = {static_cast<float>(vel(0)), 
                       static_cast<float>(vel(1)), 
                       static_cast<float>(vel(2))};
        msg.acceleration = {static_cast<float>(acc(0)), 
                           static_cast<float>(acc(1)), 
                           static_cast<float>(acc(2))};
        msg.yaw = static_cast<float>(pos(3));
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        
        return msg;
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting Multi-Waypoint Trajectory Planner..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiWaypointPlanner>());
    rclcpp::shutdown();
    return 0;
}
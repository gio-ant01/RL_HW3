#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>

using namespace std::chrono_literals;

class ForceLand : public rclcpp::Node
{
public:
    ForceLand() : Node("force_land"),
                  control_active(false),
                  ready_to_trigger(true),
                  ground_contact_detected(false),
                  altitude_enu(0.0f),
                  manual_throttle(0.0f)
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // --- SUBSCRIBERS ---

        // Altitudine
        local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position",
            qos,
            std::bind(&ForceLand::position_callback, this, std::placeholders::_1));

        // Ground contact
        land_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected",
            qos,
            std::bind(&ForceLand::land_callback, this, std::placeholders::_1));

        // Manual altitude setpoint (throttle)
        manual_sub_ = this->create_subscription<px4_msgs::msg::ManualControlSetpoint>(
            "/fmu/out/manual_control_setpoint",
            qos,
            std::bind(&ForceLand::manual_callback, this, std::placeholders::_1));

        // Publisher LAND command
        land_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        // Timer
        timer_ = this->create_wall_timer(50ms, std::bind(&ForceLand::activate_switch, this));
    }

private:
    // Subscribers & publishers
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;
    rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_sub_;

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr land_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Logic state
    bool control_active;
    bool ready_to_trigger;
    bool ground_contact_detected;

    // Data storage
    float altitude_enu;
    float manual_throttle;

    // --- CALLBACK: ALTITUDINE (ENU) ---
    void position_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
    {
        altitude_enu = -msg->z;   // NED -> ENU 
        std::cout << "[INFO] Altitude ENU = " << altitude_enu << " m" << std::endl;

        // Attiva controllo sopra 20m
        if (ready_to_trigger && altitude_enu > 20.0f) {
            control_active = true;
            ready_to_trigger = false;
            std::cout << "[Trigger] Sopra 20m: controllo attivo" << std::endl;
        }

        // Disattiva sotto 20m
        if (control_active && altitude_enu < 20.0f) {
            control_active = false;
            std::cout << "[Trigger] Sotto 20m: controllo disattivo" << std::endl;
        }
    }

    // --- CALLBACK: GROUND CONTACT ---
    void land_callback(const px4_msgs::msg::VehicleLandDetected::UniquePtr msg)
    {
        ground_contact_detected = msg->ground_contact;

        if (ground_contact_detected) {
            std::cout << "[PX4] Ground contact: sistema resettato" << std::endl;
            ready_to_trigger = true;
            control_active = false;
        }
    }

    // --- CALLBACK: MANUAL ALTITUDE SETPOINT ---
    void manual_callback(const px4_msgs::msg::ManualControlSetpoint::UniquePtr msg)
    {
        manual_throttle = msg->throttle;  // valore [-1,1]

        std::cout << "[INFO] Manual throttle setpoint = "
                  << manual_throttle << std::endl;
    }

    // --- TIMER: INVIO LAND ---
    void activate_switch()
    {
        if (control_active && !ground_contact_detected) {
            px4_msgs::msg::VehicleCommand cmd{};
            cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;

            land_pub_->publish(cmd);
            std::cout << "[ForceLand] LAND command inviato" << std::endl;
        }
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting ForceLand node..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceLand>());
    rclcpp::shutdown();
    return 0;
}

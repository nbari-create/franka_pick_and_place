#include <rclcpp/rclcpp.hpp>
#include <myworkcell_core/srv/localize_part.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

class ScanNPlan : public rclcpp::Node
{
public:
  // ── Costruttore ─────────────────────────────────────────────────────────────
  ScanNPlan() : Node("scan_n_plan",
                     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    if (!this->has_parameter("base_frame"))
      this->declare_parameter("base_frame", "world");

    vision_client_ = this->create_client<myworkcell_core::srv::LocalizePart>("localize_part");
  }

  // ── Inizializza MoveIt (chiamato DOPO rclcpp::spin in un thread separato) ──
  void setup()
  {
    // Istanzia MoveItCpp
    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(this->shared_from_this());

    // Componente di pianificazione per il gruppo "manipulator"
    planning_component_ = std::make_shared<moveit_cpp::PlanningComponent>(
        "fr3_arm", moveit_cpp_);

    // Ottieni lo stato attuale del robot
    if (!moveit_cpp_->getPlanningSceneMonitor()->requestPlanningSceneState())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get planning scene");
      return;
    }

    // Carica i parametri di pianificazione
    plan_parameters_.load(this->shared_from_this());

    RCLCPP_INFO(this->get_logger(), "MoveIt inizializzato correttamente");
  }

  // ── Localizza il pezzo e muovi il robot ─────────────────────────────────────
  void start(const std::string &base_frame)
  {
    RCLCPP_INFO(get_logger(), "Attempting to localize part");

    // Aspetta che il servizio sia disponibile (max 3 secondi)
    if (!vision_client_->wait_for_service(std::chrono::seconds(3)))
    {
      RCLCPP_ERROR(get_logger(),
          "Unable to find localize_part service. Start vision_node first.");
      return;
    }

    // Prepara e invia la richiesta
    auto request = std::make_shared<myworkcell_core::srv::LocalizePart::Request>();
    request->base_frame = base_frame;
    RCLCPP_INFO_STREAM(get_logger(),
        "Requesting pose in base frame: " << base_frame);

    auto future = vision_client_->async_send_request(request);

    // Aspetta la risposta
    if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(),
          "Failed to receive LocalizePart service response");
      return;
    }

    auto response = future.get();
    if (!response->success)
    {
      RCLCPP_ERROR(this->get_logger(), "LocalizePart service failed");
      return;
    }

    RCLCPP_INFO(this->get_logger(),
        "Part Localized: x=%.3f  y=%.3f  z=%.3f",
        response->pose.position.x,
        response->pose.position.y,
        response->pose.position.z);

    // ── Pianifica e esegui il movimento ───────────────────────────────────────
    geometry_msgs::msg::PoseStamped move_target;
    move_target.header.frame_id = base_frame;   // ← corretto: parametro locale
    move_target.pose = response->pose;

    moveit::core::RobotStatePtr start_state =
        moveit_cpp_->getCurrentState(2.0);
    planning_component_->setStartState(*start_state);

    // Nome end-effector: adatta al tuo URDF (es. "panda_hand" o "tool0")
    std::string ee_link =
        moveit_cpp_->getRobotModel()
            ->getJointModelGroup(
                planning_component_->getPlanningGroupName())
            ->getLinkModelNames()
            .back();

    planning_component_->setGoal(move_target, ee_link);

    auto plan_solution = planning_component_->plan(plan_parameters_);
    if (!plan_solution)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan");
      return;
    }

    bool success = moveit_cpp_->execute(
        "fr3_arm", plan_solution.trajectory, true);
    if (!success)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to execute trajectory");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Movimento completato!");
  }

private:
  // ── Membri ──────────────────────────────────────────────────────────────────
  rclcpp::Client<myworkcell_core::srv::LocalizePart>::SharedPtr vision_client_;

  moveit_cpp::MoveItCppPtr                              moveit_cpp_;
  moveit_cpp::PlanningComponentPtr                      planning_component_;
  moveit_cpp::PlanningComponent::PlanRequestParameters  plan_parameters_;
};

// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto app = std::make_shared<ScanNPlan>();

  std::string base_frame = app->get_parameter("base_frame").as_string();

  // Spin in background — MoveIt ha bisogno che il nodo stia girando
  std::thread worker{ [app]() { rclcpp::spin(app); } };

  // Inizializza MoveIt nel thread principale
  app->setup();

  // Aspetta 2 secondi che vision_node riceva dati
  rclcpp::sleep_for(std::chrono::seconds(2));

  app->start(base_frame);

  worker.join();
  rclcpp::shutdown();
  return 0;
}

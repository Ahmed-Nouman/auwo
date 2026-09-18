#include <memory>
#include <vector>
#include <array>
#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <utility>

#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QTimer>

#ifndef Q_MOC_RUN
  #include "rclcpp/rclcpp.hpp"
  #include "std_msgs/msg/float64_multi_array.hpp"
  #include "std_msgs/msg/string.hpp"
  #include "sensor_msgs/msg/joint_state.hpp"
  #include "rviz_common/panel.hpp"
#endif

#include <pluginlib/class_list_macros.hpp>

namespace excavator_interactive_rviz
{

// Parsed content of /excavator_twin/presets (published by twin_router_node):
//   "<name> <body> <boom> <stick> <bucket>"   one line per preset
//   "cycle <name> <name> ..."                  work cycle order
struct ParsedPresets
{
  std::vector<std::pair<std::string, std::array<double, 4>>> poses;
  std::vector<std::string> cycle;
};

inline ParsedPresets parsePresetText(const std::string & text)
{
  ParsedPresets out;
  std::istringstream lines(text);
  std::string line;
  while (std::getline(lines, line)) {
    std::istringstream ls(line);
    std::string name;
    if (!(ls >> name)) {
      continue;
    }
    if (name == "cycle") {
      std::string item;
      while (ls >> item) {
        out.cycle.push_back(item);
      }
      continue;
    }
    std::array<double, 4> q{};
    bool ok = true;
    for (auto & v : q) {
      if (!(ls >> v)) {
        ok = false;
        break;
      }
    }
    if (ok) {
      out.poses.emplace_back(name, q);
    }
  }
  return out;
}

class PresetPosesPanel : public rviz_common::Panel
{
  Q_OBJECT

  enum class Mode
  {
    Disabled = 0,
    Simulation = 1,
    RemoteControl = 2,
    Dual = 3,
  };

public:
  explicit PresetPosesPanel(QWidget * parent = nullptr)
  : rviz_common::Panel(parent),
    enabled_(false),
    current_mode_(Mode::Disabled),
    have_joint_state_(false),
    cycle_running_(false),
    cycle_index_(0)
  {
    // --- Qt UI -------------------------------------------------------------
    auto * main_layout = new QVBoxLayout;

    // Title
    title_ = new QLabel("Excavator Preset Poses");
    QFont f = title_->font();
    f.setBold(true);
    title_->setFont(f);
    main_layout->addWidget(title_);

    // Mode + Enable row
    auto * mode_layout = new QHBoxLayout;
    auto * mode_label = new QLabel("Mode:");
    mode_combo_ = new QComboBox;
    mode_combo_->addItem("Disabled");       // index 0
    mode_combo_->addItem("Simulation");     // index 1
    mode_combo_->addItem("Remote-Control"); // index 2
    mode_combo_->addItem("Dual");           // index 3

    enable_button_ = new QPushButton("Enable motion");
    enable_button_->setCheckable(true);

    mode_layout->addWidget(mode_label);
    mode_layout->addWidget(mode_combo_);
    mode_layout->addWidget(enable_button_);
    main_layout->addLayout(mode_layout);

    // Status row: mode/commands + pose + joints
    status_label_ = new QLabel("Mode: Disabled | Commands: OFF");
    pose_label_   = new QLabel("Pose: (no data)");
    joint_label_  = new QLabel("Joints: (no data)");
    status_label_->setWordWrap(true);
    pose_label_->setWordWrap(true);
    joint_label_->setWordWrap(true);

    main_layout->addWidget(status_label_);
    main_layout->addWidget(pose_label_);
    main_layout->addWidget(joint_label_);

    // Row 1: Idle, Dig, Dump, Transport
    auto * row1 = new QHBoxLayout;
    idle_button_      = new QPushButton("Idle");
    dig_button_       = new QPushButton("Dig");
    dump_button_      = new QPushButton("Dump");
    transport_button_ = new QPushButton("Transport");

    row1->addWidget(idle_button_);
    row1->addWidget(dig_button_);
    row1->addWidget(dump_button_);
    row1->addWidget(transport_button_);
    main_layout->addLayout(row1);

    // Row 2: Run/Stop Cycle
    auto * row2 = new QHBoxLayout;
    run_cycle_button_  = new QPushButton("Run Cycle");
    stop_cycle_button_ = new QPushButton("Stop Cycle");
    row2->addWidget(run_cycle_button_);
    row2->addWidget(stop_cycle_button_);
    main_layout->addLayout(row2);

    main_layout->addStretch(1);
    setLayout(main_layout);

    // --- Preset definitions -----------------------------------------------
    // joint order: [body_rotation, boom_rotation, stick_rotation, bucket_rotation]
    // These are the v1 defaults. When twin_router_node runs, the presets of the
    // selected excavator model arrive on /excavator_twin/presets and replace them.
    presets_.push_back({"Idle",      {-0.339,  -0.639,  -0.809,  -0.660}});
    presets_.push_back({"Dig",       {-3.192,  -0.509,  -1.760,  -1.690}});
    presets_.push_back({"Dump",      {0.611,  -1.210,  -1.760,  -1.632}});
    presets_.push_back({"Transport", {0.0,  -1.2,  -1.2,  -1.5}});

    // Work cycle: Idle -> Dig -> Transport -> Dump (loop)
    cycle_sequence_ = {0, 3, 1, 3, 2, 3};

    // --- ROS 2 node + pub/sub ---------------------------------------------
    node_ = rclcpp::Node::make_shared("excavator_preset_panel");

    // Unified twin command publisher
    twin_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/excavator_twin/commands", 10);

    // Mode command publisher (to TwinRouter)
    twin_mode_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/excavator_twin/mode_cmd", 10);

    // Unified twin state subscription
    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/excavator_twin/state", 10,
      std::bind(&PresetPosesPanel::onJointState, this, std::placeholders::_1));

    // Model-specific presets (latched topics from twin_router_node)
    auto latched = rclcpp::QoS(1).transient_local().reliable();
    presets_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/excavator_twin/presets", latched,
      std::bind(&PresetPosesPanel::onPresets, this, std::placeholders::_1));
    model_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/excavator_twin/model", latched,
      std::bind(&PresetPosesPanel::onModel, this, std::placeholders::_1));

    // Timer to spin the node (so joint_states callbacks run)
    ros_timer_ = new QTimer(this);
    connect(ros_timer_, SIGNAL(timeout()), this, SLOT(onRosTimer()));
    ros_timer_->start(50);  // 20 Hz

    // Cycle timer
    cycle_timer_ = new QTimer(this);
    connect(cycle_timer_, SIGNAL(timeout()), this, SLOT(onCycleTimer()));
    cycle_timer_->setInterval(2000);  // 2 seconds per pose

    // --- Connect UI signals -----------------------------------------------
    connect(mode_combo_, SIGNAL(currentIndexChanged(int)),
            this, SLOT(onModeChanged(int)));
    connect(enable_button_, SIGNAL(clicked()),
            this, SLOT(onEnableClicked()));

    connect(idle_button_,      SIGNAL(clicked()), this, SLOT(onIdleClicked()));
    connect(dig_button_,       SIGNAL(clicked()), this, SLOT(onDigClicked()));
    connect(dump_button_,      SIGNAL(clicked()), this, SLOT(onDumpClicked()));
    connect(transport_button_, SIGNAL(clicked()), this, SLOT(onTransportClicked()));

    connect(run_cycle_button_,  SIGNAL(clicked()), this, SLOT(onRunCycleClicked()));
    connect(stop_cycle_button_, SIGNAL(clicked()), this, SLOT(onStopCycleClicked()));

    updateStatusLabel();
    publishRouterMode();
  }

private Q_SLOTS:
  // --- Mode + Enable -------------------------------------------------------

  void onModeChanged(int index)
  {
    switch (index) {
      case 0: current_mode_ = Mode::Disabled;      break;
      case 1: current_mode_ = Mode::Simulation;    break;
      case 2: current_mode_ = Mode::RemoteControl; break;
      case 3: current_mode_ = Mode::Dual;          break;
      default: current_mode_ = Mode::Disabled;     break;
    }
    updateStatusLabel();
    publishRouterMode();
  }

  void onEnableClicked()
  {
    enabled_ = enable_button_->isChecked();

    if (enabled_) {
      // If motion is enabled while in Disabled mode,
      // automatically switch to Simulation mode.
      if (current_mode_ == Mode::Disabled) {
        // 0: Disabled, 1: Simulation, 2: Remote-Control, 3: Dual
        mode_combo_->setCurrentIndex(1);  // Simulation
        // onModeChanged(1) will update current_mode_
      }
      enable_button_->setText("Motion ENABLED");
    } else {
      // Turning motion off:
      //  - stop any running cycles
      //  - switch the mode to Disabled
      cycle_running_ = false;
      cycle_timer_->stop();

      mode_combo_->setCurrentIndex(0);  // Disabled
      // onModeChanged(0) will update current_mode_

      enable_button_->setText("Enable motion");
    }

    updateStatusLabel();
    publishRouterMode();
  }

  // --- Preset buttons ------------------------------------------------------

  void onIdleClicked()      { sendPresetByName("Idle"); }
  void onDigClicked()       { sendPresetByName("Dig"); }
  void onDumpClicked()      { sendPresetByName("Dump"); }
  void onTransportClicked() { sendPresetByName("Transport"); }

  // --- Work cycle buttons --------------------------------------------------

  void onRunCycleClicked()
  {
    if (!enabled_ || current_mode_ == Mode::Disabled) {
      return;
    }
    cycle_running_ = true;
    cycle_index_ = 0;
    cycle_timer_->start();
    // Send first pose immediately
    sendPresetByIndex(cycle_sequence_[cycle_index_]);
    cycle_index_ = (cycle_index_ + 1) % static_cast<int>(cycle_sequence_.size());
  }

  void onStopCycleClicked()
  {
    cycle_running_ = false;
    cycle_timer_->stop();
  }

  // --- ROS timers / callbacks ---------------------------------------------

  void onRosTimer()
  {
    if (rclcpp::ok()) {
      rclcpp::spin_some(node_);
    }
  }

  void onCycleTimer()
  {
    if (!cycle_running_ || !enabled_ || current_mode_ == Mode::Disabled) {
      return;
    }
    sendPresetByIndex(cycle_sequence_[cycle_index_]);
    cycle_index_ = (cycle_index_ + 1) % static_cast<int>(cycle_sequence_.size());
  }

private:
  // --- Joint state handling -----------------------------------------------

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Map name->position
    std::map<std::string, double> m;
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      m[msg->name[i]] = msg->position[i];
    }

    // Extract in the order we care about
    const char * names[4] = {
      "body_rotation",
      "boom_rotation",
      "stick_rotation",
      "bucket_rotation"
    };

    for (size_t i = 0; i < 4; ++i) {
      auto it = m.find(names[i]);
      if (it != m.end()) {
        current_joints_[i] = it->second;
      }
      // else keep previous value
    }

    have_joint_state_ = true;
    updatePoseStatus();
  }

  // --- Model presets -------------------------------------------------------

  void onModel(const std_msgs::msg::String::SharedPtr msg)
  {
    model_name_ = QString::fromStdString(msg->data);
    title_->setText(QString("Excavator Preset Poses (model %1)").arg(model_name_));
  }

  void onPresets(const std_msgs::msg::String::SharedPtr msg)
  {
    const ParsedPresets parsed = parsePresetText(msg->data);
    if (parsed.poses.empty()) {
      return;
    }
    std::vector<Preset> presets;
    for (const auto & p : parsed.poses) {
      presets.push_back({QString::fromStdString(p.first), p.second});
    }
    std::vector<int> cycle;
    for (const auto & name : parsed.cycle) {
      for (size_t i = 0; i < presets.size(); ++i) {
        if (presets[i].name == QString::fromStdString(name)) {
          cycle.push_back(static_cast<int>(i));
          break;
        }
      }
    }
    presets_ = presets;
    if (!cycle.empty()) {
      cycle_sequence_ = cycle;
    }
    cycle_index_ = 0;
    updatePoseStatus();
  }

  // --- Helpers -------------------------------------------------------------

  struct Preset
  {
    QString name;
    std::array<double, 4> joints;
  };

  void sendPresetByName(const QString & name)
  {
    for (size_t i = 0; i < presets_.size(); ++i) {
      if (presets_[i].name == name) {
        sendPresetByIndex(static_cast<int>(i));
        return;
      }
    }
  }

  void sendPresetByIndex(int idx)
  {
    if (idx < 0 || idx >= static_cast<int>(presets_.size())) {
      return;
    }
    sendPose(presets_[idx].joints);
  }

  void sendPose(const std::array<double, 4> & joints)
  {
    // Safety gate: if disabled or in Disabled mode, do nothing
    if (!enabled_ || current_mode_ == Mode::Disabled) {
      return;
    }

    if (!rclcpp::ok()) {
      return;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(joints.begin(), joints.end());

    // Always publish to the unified twin commands topic.
    // TwinRouterNode will decide whether this goes to sim, physical, or both.
    twin_cmd_pub_->publish(msg);
  }

  void updateStatusLabel()
  {
    QString mode_str;
    switch (current_mode_) {
      case Mode::Disabled:      mode_str = "Disabled";       break;
      case Mode::Simulation:    mode_str = "Simulation";     break;
      case Mode::RemoteControl: mode_str = "Remote-Control"; break;
      case Mode::Dual:          mode_str = "Dual";           break;
    }

    QString cmd_str = (enabled_ && current_mode_ != Mode::Disabled)
                      ? "ON" : "OFF";

    status_label_->setText(
      QString("Mode: %1 | Commands: %2").arg(mode_str, cmd_str));
  }

  void updatePoseStatus()
  {
    if (!have_joint_state_) {
      pose_label_->setText("Pose: (no joint data yet)");
      joint_label_->setText("Joints: (no joint data yet)");
      return;
    }

    // Format joints
    QString joints_str = QString("Joints [body, boom, stick, bucket]: "
                                 "%1, %2, %3, %4")
      .arg(current_joints_[0], 0, 'f', 3)
      .arg(current_joints_[1], 0, 'f', 3)
      .arg(current_joints_[2], 0, 'f', 3)
      .arg(current_joints_[3], 0, 'f', 3);
    joint_label_->setText(joints_str);

    // Find closest preset
    double best_max_diff = 1e9;
    QString best_name = "Custom";

    for (const auto & p : presets_) {
      double max_diff = 0.0;
      for (size_t i = 0; i < 4; ++i) {
        double d = std::fabs(current_joints_[i] - p.joints[i]);
        if (d > max_diff) max_diff = d;
      }
      if (max_diff < best_max_diff) {
        best_max_diff = max_diff;
        best_name = p.name;
      }
    }

    // Threshold: if max joint difference < 0.25 rad, treat as "≈ preset"
    if (best_max_diff < 0.25) {
      pose_label_->setText(QString("Pose: \u2248 %1 (max diff ~ %2 rad)")
                           .arg(best_name)
                           .arg(best_max_diff, 0, 'f', 3));
    } else {
      pose_label_->setText(QString("Pose: Custom (closest: %1, max diff ~ %2 rad)")
                           .arg(best_name)
                           .arg(best_max_diff, 0, 'f', 3));
    }
  }

  void publishRouterMode()
  {
    if (!rclcpp::ok()) {
      return;
    }

    // Map panel state to router mode string
    std::string m = "disabled";
    if (!enabled_) {
      m = "disabled";
    } else {
      switch (current_mode_) {
        case Mode::Disabled:      m = "disabled";   break;
        case Mode::Simulation:    m = "simulation"; break;
        case Mode::RemoteControl: m = "physical";   break;
        case Mode::Dual:          m = "dual";       break;
      }
    }

    std_msgs::msg::String msg;
    msg.data = m;
    twin_mode_pub_->publish(msg);
  }

  // --- Members -------------------------------------------------------------

  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr twin_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr twin_mode_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr presets_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr model_sub_;
  QString model_name_;

  QTimer * ros_timer_;
  QTimer * cycle_timer_;

  // Presets and work cycle
  std::vector<Preset> presets_;
  std::vector<int> cycle_sequence_;

  // Current joint state
  std::array<double, 4> current_joints_{{0.0, 0.0, 0.0, 0.0}};
  bool have_joint_state_;

  // Mode & safety
  bool enabled_;
  Mode current_mode_;

  // Cycle state
  bool cycle_running_;
  int cycle_index_;

  // UI elements
  QLabel * title_;
  QComboBox * mode_combo_;
  QPushButton * enable_button_;
  QLabel * status_label_;
  QLabel * pose_label_;
  QLabel * joint_label_;

  QPushButton * idle_button_;
  QPushButton * dig_button_;
  QPushButton * dump_button_;
  QPushButton * transport_button_;
  QPushButton * run_cycle_button_;
  QPushButton * stop_cycle_button_;
};

}  // namespace excavator_interactive_rviz

// moc include for Q_OBJECT
#include "preset_poses_panel.moc"

// Export the panel to RViz
PLUGINLIB_EXPORT_CLASS(
  excavator_interactive_rviz::PresetPosesPanel,
  rviz_common::Panel)


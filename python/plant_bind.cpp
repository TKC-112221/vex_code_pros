/* ---------------------------------------------------------------------------
 * plant_bind.cpp  --  pybind11 bridge to MockDrive for the training env.
 *
 * The module owns a VASController and a MockDrive. step() is one 10 ms
 * plant update: applyDrive(surge, sway, yaw) then stepSimulation. It does
 * not open the field window. read_policy() is DrivePolicy::read, so the
 * observation vector matches the firmware actor.
 * ------------------------------------------------------------------------- */
#include "control/DriftFollower.h"
#include "control/DrivePolicy.h"
#include "control/VASController.h"
#include "core/LogFrame.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include "hardware/MockDrive.h"
#include "hardware/MockDriveParams.h"
#include "math/NeuralNetwork.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdio>
#include <string>
#include <vector>

namespace py = pybind11;

namespace {

// Same armature model main.cpp binds into the host plant. stepSimulation
// does not read it. applyDrive's current limit does, and that limit is off.
VASController makeFeedforward() {
    return VASController(Feedforward::KS_LIN, Feedforward::KV_LIN, Feedforward::KA_LIN,
                         Feedforward::KS_ANG, Feedforward::KV_ANG, Feedforward::KA_ANG);
}

class PlantSim {
public:
    PlantSim() : feedforward_(makeFeedforward()), drive_(feedforward_, System::MAX_VOLTS, false) {
        drive_.initialize();
    }

    void reset(double x_in, double y_in, double theta,
               double v_lin, double v_lat, double v_ang) {
        drive_.setPlant(x_in, y_in, theta, v_lin, v_lat, v_ang);
    }

    // Chassis volts on the universal command axes. sway is stored and
    // ignored by the differential plant, matching TankDrive.
    MockDrive::PlantSnapshot step(double surge, double sway, double yaw) {
        ChassisCommand cmd;
        cmd.linear = surge;
        cmd.lateral = sway;
        cmd.angular = yaw;
        drive_.applyDrive(cmd, ChassisObservation{});
        drive_.stepSimulation(System::DT_SEC);
        return drive_.plantSnapshot();
    }

    MockDrive::PlantSnapshot state() const { return drive_.plantSnapshot(); }

    void setDynamics(double velocity_tau, double skid_tau, double resistance, double speed_scale) {
        drive_.setDynamics(velocity_tau, skid_tau, resistance, speed_scale);
    }

private:
    VASController feedforward_;
    MockDrive drive_;
};

StateEstimate estimateFrom(double x_in, double y_in, double theta, double v_lin, double v_ang) {
    StateEstimate state;
    state.pose.x = x_in;
    state.pose.y = y_in;
    state.pose.theta = theta;
    state.v_lin = v_lin;
    state.v_ang = v_ang;
    return state;
}

DriftCurveShape shapeFrom(const std::string& name) {
    if (name == "Arc") return DriftCurveShape::Arc;
    if (name == "Exponential") return DriftCurveShape::Exponential;
    if (name == "Quartic") return DriftCurveShape::Quartic;
    if (name == "Quintic") return DriftCurveShape::Quintic;
    if (name == "Hermite") return DriftCurveShape::Hermite;
    if (name == "QuarticHermite") return DriftCurveShape::QuarticHermite;
    return DriftCurveShape::Hermite;
}

DriftCurveTarget targetFrom(double x_in, double y_in, double exit_direction, double exit_speed,
                            bool backwards, bool ignore_direction) {
    DriftCurveTarget target;
    target.x = x_in;
    target.y = y_in;
    target.exit_direction = exit_direction;
    target.exit_speed = exit_speed;
    target.backwards = backwards;
    target.ignore_direction = ignore_direction;
    return target;
}

// One planned curve, then the same follower the pose override runs.
// begin() drops the plan. The next command() fits it from that pose.
class DriftGuide {
public:
    DriftGuide() : feedforward_(makeFeedforward()) {
        follower_.bindFeedforward(&feedforward_);
    }

    DriftGuide(const DriftGuide&) = delete;
    DriftGuide& operator=(const DriftGuide&) = delete;

    void begin(double target_x, double target_y, double exit_direction, double exit_speed,
               bool backwards, bool ignore_direction, const std::string& shape) {
        follower_.config.shape = shapeFrom(shape);
        follower_.reset();
        follower_.config.forwards = !backwards;
        follower_.target = targetFrom(target_x, target_y, exit_direction, exit_speed,
                                      backwards, ignore_direction);
        // Negative means "unset", so the follower uses config.path_accel.
        if (max_accel_ >= 0.0) follower_.target.max_accel = max_accel_;
        follower_.bindFeedforward(&feedforward_);
    }

    // One candidate from the tuner. Names match include/core/Tuning.h.
    void configure(double ks_lin, double kv_lin, double ka_lin,
                   double ks_ang, double kv_ang, double ka_ang,
                   double exp_rate, double close_m, double drift_accel, double max_slip,
                   double path_accel, double k_slip, double a_max,
                   double lat_kp, double lat_ki, double lat_kd, double lat_kd2, double lat_i_limit,
                   double head_kp, double head_ki, double head_kd, double head_windup,
                   double settle_speed, double min_accel, double turn_radius_max, double kappa_eps,
                   double settle_in, double settle_deg, double max_accel) {
        feedforward_.setGains(ks_lin, kv_lin, ka_lin, ks_ang, kv_ang, ka_ang);
        follower_.bindFeedforward(&feedforward_);
        auto& c = follower_.config;
        c.exp_rate = exp_rate;
        c.close_m = close_m;
        c.drift_accel = drift_accel;
        c.max_slip_rad = max_slip;
        c.path_accel = path_accel;
        c.k_slip = k_slip;
        c.a_max = a_max;
        c.lat_kP = lat_kp;
        c.lat_kI = lat_ki;
        c.lat_kD = lat_kd;
        c.lat_kD2 = lat_kd2;
        c.lat_i_limit = lat_i_limit;
        c.head_kP = head_kp;
        c.head_kI = head_ki;
        c.head_kD = head_kd;
        c.head_windup = head_windup;
        c.settle_speed = settle_speed;
        c.min_accel = min_accel;
        c.turn_radius_max = turn_radius_max;
        c.kappa_eps = kappa_eps;
        c.settle_in = settle_in;
        c.settle_deg = settle_deg;
        max_accel_ = max_accel;
    }

    std::tuple<double, double, double> command(double x_in, double y_in, double theta,
                                               double v_lin, double v_ang) {
        const ChassisCommand cmd = follower_.compute(estimateFrom(x_in, y_in, theta, v_lin, v_ang));
        return {cmd.linear, cmd.lateral, cmd.angular};
    }

private:
    VASController feedforward_;
    DriftFollower follower_;
    double max_accel_ = Practice::MAX_ACCEL_MPS2;
};

// Raw LogFrame file written by LogFrameReplayer (skills_auto.bin / prematch.bin).
std::vector<py::dict> readLog(const std::string& path) {
    std::vector<py::dict> out;
    FILE* file = nullptr;
    if (fopen_s(&file, path.c_str(), "rb") != 0 || file == nullptr) return out;
    LogFrame frame;
    while (std::fread(&frame, sizeof(LogFrame), 1, file) == 1) {
        py::dict row;
        row["time"] = frame.time;
        row["applied_linear"] = frame.applied_linear;
        row["applied_lateral"] = frame.applied_lateral;
        row["applied_angular"] = frame.applied_angular;
        row["v_lin"] = frame.v_lin;
        row["v_ang"] = frame.v_ang;
        row["v_lat"] = frame.v_inertial_lat;
        row["left_amps"] = frame.l_current_amps;
        row["right_amps"] = frame.r_current_amps;
        row["x_in"] = frame.x;
        row["y_in"] = frame.y;
        row["theta"] = frame.theta;
        out.push_back(std::move(row));
    }
    std::fclose(file);
    return out;
}

} // namespace

PYBIND11_MODULE(plant, m) {
    m.doc() = "MockDrive plant and the drive-policy feature packing";

    py::class_<MockDrive::PlantSnapshot>(m, "PlantSnapshot")
        .def_readonly("x_in", &MockDrive::PlantSnapshot::x_in)
        .def_readonly("y_in", &MockDrive::PlantSnapshot::y_in)
        .def_readonly("theta", &MockDrive::PlantSnapshot::theta)
        .def_readonly("v_lin", &MockDrive::PlantSnapshot::v_lin)
        .def_readonly("v_lat", &MockDrive::PlantSnapshot::v_lat)
        .def_readonly("v_ang", &MockDrive::PlantSnapshot::v_ang)
        .def_readonly("left_amps", &MockDrive::PlantSnapshot::left_amps)
        .def_readonly("right_amps", &MockDrive::PlantSnapshot::right_amps);

    py::class_<DriftGuide>(m, "DriftGuide")
        .def(py::init<>())
        .def("begin", &DriftGuide::begin,
             py::arg("target_x"), py::arg("target_y"), py::arg("exit_direction"),
             py::arg("exit_speed"), py::arg("backwards") = false, py::arg("ignore_direction") = false,
             py::arg("shape") = "Hermite")
        .def("command", &DriftGuide::command,
             py::arg("x_in"), py::arg("y_in"), py::arg("theta"), py::arg("v_lin"), py::arg("v_ang"))
        .def("configure", &DriftGuide::configure,
             py::arg("ks_lin"), py::arg("kv_lin"), py::arg("ka_lin"),
             py::arg("ks_ang"), py::arg("kv_ang"), py::arg("ka_ang"),
             py::arg("exp_rate"), py::arg("close_m"), py::arg("drift_accel"), py::arg("max_slip"),
             py::arg("path_accel"), py::arg("k_slip"), py::arg("a_max"),
             py::arg("lat_kp"), py::arg("lat_ki"), py::arg("lat_kd"), py::arg("lat_kd2"),
             py::arg("lat_i_limit"),
             py::arg("head_kp"), py::arg("head_ki"), py::arg("head_kd"), py::arg("head_windup"),
             py::arg("settle_speed"), py::arg("min_accel"), py::arg("turn_radius_max"),
             py::arg("kappa_eps"), py::arg("settle_in"), py::arg("settle_deg"), py::arg("max_accel"));

    py::class_<PlantSim>(m, "PlantSim")
        .def(py::init<>())
        .def("reset", &PlantSim::reset,
             py::arg("x_in"), py::arg("y_in"), py::arg("theta"),
             py::arg("v_lin") = 0.0, py::arg("v_lat") = 0.0, py::arg("v_ang") = 0.0)
        .def("step", &PlantSim::step, py::arg("surge"), py::arg("sway"), py::arg("yaw"))
        .def("state", &PlantSim::state)
        .def("set_dynamics", &PlantSim::setDynamics,
             py::arg("velocity_tau"), py::arg("skid_tau"),
             py::arg("resistance"), py::arg("speed_scale"));

    py::class_<DrivePolicy::Reading>(m, "Reading")
        .def_property_readonly("features", [](const DrivePolicy::Reading& reading) {
            return std::vector<float>(reading.features, reading.features + DrivePolicy::kFeatures);
        })
        .def_readonly("distance_in", &DrivePolicy::Reading::distance_in)
        .def_readonly("travel_mps", &DrivePolicy::Reading::travel_mps)
        .def_readonly("heading_error_rad", &DrivePolicy::Reading::heading_error_rad)
        .def_readonly("at_target", &DrivePolicy::Reading::at_target);

    m.def("read_policy", [](double prev_surge, double prev_sway, double prev_yaw,
                            double path_phi, double vel_heading, double dphi_ds, double d2phi_ds2,
                            double target_thrust, double cross_track_in) {
        DrivePolicy::FollowInput in;
        in.prev_surge = prev_surge;
        in.prev_sway = prev_sway;
        in.prev_yaw = prev_yaw;
        in.path_phi = path_phi;
        in.vel_heading = vel_heading;
        in.dphi_ds = dphi_ds;
        in.d2phi_ds2 = d2phi_ds2;
        in.target_thrust = target_thrust;
        in.cross_track_in = cross_track_in;
        return DrivePolicy::read(in);
    }, py::arg("prev_surge"), py::arg("prev_sway"), py::arg("prev_yaw"),
       py::arg("path_phi"), py::arg("vel_heading"), py::arg("dphi_ds"), py::arg("d2phi_ds2"),
       py::arg("target_thrust"), py::arg("cross_track_in"));

    m.def("command", [](double prev_surge, double prev_sway, double prev_yaw,
                        double path_phi, double vel_heading, double dphi_ds, double d2phi_ds2,
                        double target_thrust, double cross_track_in) {
        DrivePolicy::FollowInput in;
        in.prev_surge = prev_surge;
        in.prev_sway = prev_sway;
        in.prev_yaw = prev_yaw;
        in.path_phi = path_phi;
        in.vel_heading = vel_heading;
        in.dphi_ds = dphi_ds;
        in.d2phi_ds2 = d2phi_ds2;
        in.target_thrust = target_thrust;
        in.cross_track_in = cross_track_in;
        const CommandVector cmd = DrivePolicy::command(in);
        return py::make_tuple(cmd.surge, cmd.sway, cmd.yaw);
    }, py::arg("prev_surge"), py::arg("prev_sway"), py::arg("prev_yaw"),
       py::arg("path_phi"), py::arg("vel_heading"), py::arg("dphi_ds"), py::arg("d2phi_ds2"),
       py::arg("target_thrust"), py::arg("cross_track_in"));

    m.def("assign_weights", [](const std::vector<float>& weights) {
        return DrivePolicy::assignWeights(weights.data(), static_cast<int>(weights.size()));
    });

    m.def("read_log", &readLog, py::arg("path"));
    m.attr("LOG_FRAME_BYTES") = static_cast<int>(sizeof(LogFrame));
    m.attr("DT_SEC") = System::DT_SEC;
    m.attr("VELOCITY_TAU") = MockPlant::VELOCITY_TAU_SEC;
    m.attr("SKID_TAU") = MockPlant::SKID_TAU_SEC;
    m.attr("SIDE_RESISTANCE") = MockPlant::SIDE_RESISTANCE_OHM;
    m.attr("SPEED_SCALE") = MockPlant::SPEED_SCALE;
    m.attr("KS_LIN") = Feedforward::KS_LIN;
    m.attr("KV_LIN") = Feedforward::KV_LIN;
    m.attr("KA_LIN") = Feedforward::KA_LIN;
    m.attr("KS_ANG") = Feedforward::KS_ANG;
    m.attr("KV_ANG") = Feedforward::KV_ANG;
    m.attr("KA_ANG") = Feedforward::KA_ANG;
    m.attr("LIN_SPEED_AT_SAT") = Feedforward::LIN_SPEED_AT_SAT;
    m.attr("LIN_ACCEL_AT_FULL_V") = Feedforward::LIN_ACCEL_AT_FULL_V;
    m.attr("ANG_RATE_PER_VOLT") = Feedforward::ANG_RATE_PER_VOLT;
    m.attr("ANG_LEVER_M") = Feedforward::ANG_LEVER_M;
    m.attr("ANG_KA_COEFF") = Feedforward::ANG_KA_COEFF;
    m.attr("ANG_KA_LENGTH_M") = Feedforward::ANG_KA_LENGTH_M;
    m.attr("ANG_KA_MOTOR_COUNT") = Feedforward::ANG_KA_MOTOR_COUNT;
    m.attr("EXP_RATE") = Drift::EXP_RATE;
    m.attr("DRIFT_ACCEL") = Drift::DRIFT_ACCEL;
    m.attr("PATH_ACCEL") = Drift::PATH_ACCEL;
    m.attr("MAX_SLIP_RAD") = Drift::MAX_SLIP_RAD;
    m.attr("K_SLIP") = Drift::K_SLIP;
    m.attr("A_MAX") = Drift::A_MAX;
    m.attr("LAT_KP") = Drift::LAT_KP;
    m.attr("LAT_KI") = Drift::LAT_KI;
    m.attr("LAT_KD") = Drift::LAT_KD;
    m.attr("LAT_KD2") = Drift::LAT_KD2;
    m.attr("LAT_I_LIMIT") = Drift::LAT_I_LIMIT;
    m.attr("CLOSE_IN") = Boomerang::CLOSE_IN;
    m.attr("LATERAL_AXIS") = py::make_tuple(
        Boomerang::Lateral.kP, Boomerang::Lateral.kI, Boomerang::Lateral.kD,
        Boomerang::Lateral.windup, Boomerang::Lateral.small_error,
        Boomerang::Lateral.small_ms, Boomerang::Lateral.large_error,
        Boomerang::Lateral.large_ms, Boomerang::Lateral.slew);
    m.attr("ANGULAR_AXIS") = py::make_tuple(
        Boomerang::Angular.kP, Boomerang::Angular.kI, Boomerang::Angular.kD,
        Boomerang::Angular.windup, Boomerang::Angular.small_error,
        Boomerang::Angular.small_ms, Boomerang::Angular.large_error,
        Boomerang::Angular.large_ms, Boomerang::Angular.slew);
    m.attr("HEAD_KP") = Boomerang::Angular.kP;
    m.attr("HEAD_KI") = Boomerang::Angular.kI;
    m.attr("HEAD_KD") = Boomerang::Angular.kD;
    m.attr("HEAD_WINDUP") = Boomerang::Angular.windup;
    m.attr("SETTLE_IN") = Boomerang::Lateral.small_error;
    m.attr("SETTLE_DEG") = Boomerang::Angular.small_error;
    m.attr("SETTLE_SPEED") = Motion::SETTLE_SPEED_MPS;
    m.attr("MIN_ACCEL") = Motion::MIN_ACCEL_MPS2;
    m.attr("KAPPA_EPS") = Motion::KAPPA_EPS;
    m.attr("TURN_RADIUS_MAX") = Physics::TURN_RADIUS_MAX_M;
    m.attr("PRACTICE_ACCEL") = Practice::MAX_ACCEL_MPS2;
    m.attr("METERS_PER_INCH") = kMetersPerInch;
    m.attr("MAX_VOLTS") = System::MAX_VOLTS;
    m.attr("MAX_VEL_LIN") = Robot::MAX_VEL_LIN;
    m.attr("FIELD_HALF_IN") = metersToInches(Robot::Range::FIELD_HALF_M);
    m.attr("FEATURE_COUNT") = DrivePolicy::kFeatures;
    m.attr("KAPPA_UNIT_PER_IN") = DrivePolicy::kKappaUnitPerIn;
    m.attr("KAPPA_RATE_UNIT") = DrivePolicy::kKappaRateUnit;
    m.attr("CROSS_UNIT_IN") = DrivePolicy::kCrossUnitIn;
    m.attr("ACTION_COUNT") = DrivePolicy::kOutputs;
    m.attr("LAYER_SIZES") = py::cast(std::vector<int>(
        DrivePolicy::kLayerSizes, DrivePolicy::kLayerSizes + DrivePolicy::kLayerCount));
    m.attr("WEIGHT_COUNT") = NeuralNetwork::countWeights(DrivePolicy::kLayerSizes,
                                                         DrivePolicy::kLayerCount);
    m.attr("POSITION_TOL_IN") = DrivePolicy::kPositionToleranceIn;
    m.attr("SPEED_TOL_MPS") = DrivePolicy::kSpeedToleranceMps;
    m.attr("HEADING_TOL_RAD") = DrivePolicy::kHeadingToleranceRad;
    m.attr("DEPART_MARGIN_IN") = DrivePolicy::kDepartMarginIn;
    m.attr("PATH_SHAPES") = py::cast(std::vector<std::string>{
        "Arc", "Exponential", "Quartic", "Quintic", "Hermite", "QuarticHermite"});
}

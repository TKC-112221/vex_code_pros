/* ---------------------------------------------------------------------------
 * NeuralFeedforward.cpp  --  chassis feature packing + SD-card weight load.
 * The forward pass itself is math/NeuralNetwork. Weight file order is the
 * linear network, then the angular network, each packed as NeuralNetwork
 * documents (row-major weights[out][in], then biases, per layer).
 * ------------------------------------------------------------------------- */
#include "control/NeuralFeedforward.h"
#include "math/MathUtils.h"
#include "platform/RobotFile.h"
#include <cmath>
#include <cstdio>

// Topologies recorded in data/generated_limits.h by utils/train_dynamics.py.
//   linear  hidden [64, 32]       tanh
//   angular hidden [128, 64]      leaky ReLU   (search result "Ang A")
// "Ang B" [256, 128, 64] leaky ReLU is the same NeuralNetwork with a longer
// size list; swap kAngularSizes if that export is the one on the card.
static constexpr int kLinearSizes[]  = {7, 64, 32, 1};
static constexpr int kAngularSizes[] = {6, 128, 64, 1};
static constexpr int kLinearDepth =
    static_cast<int>(sizeof(kLinearSizes) / sizeof(kLinearSizes[0]));
static constexpr int kAngularDepth =
    static_cast<int>(sizeof(kAngularSizes) / sizeof(kAngularSizes[0]));

static_assert(kLinearSizes[0] == 7, "linear feature width");
static_assert(kLinearSizes[kLinearDepth - 1] == 1, "linear model is scalar");
static_assert(kAngularSizes[0] == 6, "angular feature width");
static_assert(kAngularSizes[kAngularDepth - 1] == 1, "angular model is scalar");

// Identified operating limits. Written by init(); other code may read them.
double g_max_vel = 0.0;
double g_max_accel = 0.0;

namespace {

float directionSign(double v) {
    return (std::fabs(v) < 0.02) ? 0.0f : static_cast<float>(mathutil::sign(v));
}

} // namespace

NeuralFeedforward::NeuralFeedforward()
    : linear_(kLinearSizes, kLinearDepth, NeuralNetwork::Activation::TANH),
      angular_(kAngularSizes, kAngularDepth, NeuralNetwork::Activation::LEAKY_RELU) {}

void NeuralFeedforward::init() {
    FILE* limit_file = openRobotFile("/usd/limits.bin", "rb");
    if (limit_file) {
        fread(&g_max_vel, sizeof(double), 1, limit_file);
        fread(&g_max_accel, sizeof(double), 1, limit_file);
        fclose(limit_file);
    } else {
        printf("Error: Failed to load /usd/limits.bin\n");
    }

    FILE* mlp_file = openRobotFile("/usd/mlp_weights.bin", "rb");
    if (!mlp_file) {
        printf("Error: Failed to load /usd/mlp_weights.bin\n");
        return;
    }

    const int floats = linear_.weightCount() + angular_.weightCount();
    const long expect = static_cast<long>(floats) * static_cast<long>(sizeof(float));

    bool sized = (fseek(mlp_file, 0, SEEK_END) == 0);
    long bytes = sized ? ftell(mlp_file) : -1L;
    sized = sized && bytes >= 0 && (fseek(mlp_file, 0, SEEK_SET) == 0);

    if (!sized || bytes != expect) {
        printf("Error: mlp_weights.bin is %ld bytes, expected %ld (%d floats)\n",
               bytes, expect, floats);
    } else if (!linear_.loadWeights(mlp_file) || !angular_.loadWeights(mlp_file)) {
        printf("Error: failed to read /usd/mlp_weights.bin\n");
    }
    fclose(mlp_file);
}

double NeuralFeedforward::linearVolts(double velocity, double accel, double abs_lin_vel,
                                      double abs_lin_accel, double omega, double turn_radius) const {
    float x[7];
    x[0] = static_cast<float>(velocity);
    x[1] = static_cast<float>(accel);
    x[2] = directionSign(velocity);
    x[3] = static_cast<float>(std::fabs(abs_lin_vel));
    x[4] = static_cast<float>(std::fabs(abs_lin_accel));
    x[5] = static_cast<float>(std::fabs(omega));
    x[6] = static_cast<float>(turn_radius);
    return static_cast<double>(linear_.predictScalar(x));
}

double NeuralFeedforward::angularVolts(double omega, double alpha, double abs_lin_vel,
                                       double abs_lin_accel, double turn_radius) const {
    float x[6];
    x[0] = static_cast<float>(omega);
    x[1] = static_cast<float>(alpha);
    x[2] = directionSign(omega);
    x[3] = static_cast<float>(std::fabs(abs_lin_vel));
    x[4] = static_cast<float>(std::fabs(abs_lin_accel));
    x[5] = static_cast<float>(turn_radius);
    return static_cast<double>(angular_.predictScalar(x));
}

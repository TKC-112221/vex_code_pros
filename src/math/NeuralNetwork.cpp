/* ---------------------------------------------------------------------------
 * NeuralNetwork.cpp  --  MLP forward pass. No training, no file-path policy;
 * callers open the weight file and hand this class the floats.
 * ------------------------------------------------------------------------- */
#include "math/NeuralNetwork.h"

#include <cmath>
#include <cstring>
#include <new>

NeuralNetwork::NeuralNetwork(const int* layer_sizes, int num_layers,
                             Activation activation, float leaky_slope)
    : sizes_{},
      num_layers_(0),
      activation_(activation),
      leaky_slope_(leaky_slope),
      weight_count_(0),
      weights_(nullptr),
      valid_(false),
      scratch_a_{},
      scratch_b_{} {
    if (layer_sizes == nullptr || num_layers < 2 || num_layers > kMaxLayers) return;

    for (int i = 0; i < num_layers; ++i) {
        if (layer_sizes[i] < 1 || layer_sizes[i] > kMaxUnits) return;
        sizes_[i] = layer_sizes[i];
    }
    num_layers_ = num_layers;
    weight_count_ = countWeights(sizes_, num_layers_);
    valid_ = weight_count_ > 0;
}

NeuralNetwork::~NeuralNetwork() {
    delete[] weights_;
}

bool NeuralNetwork::assignWeights(const float* weights, int count) {
    if (!valid_ || weights == nullptr || count != weight_count_) return false;

    float* buf = new (std::nothrow) float[static_cast<size_t>(weight_count_)];
    if (buf == nullptr) return false;
    std::memcpy(buf, weights, static_cast<size_t>(weight_count_) * sizeof(float));

    delete[] weights_;
    weights_ = buf;
    return true;
}

bool NeuralNetwork::loadWeights(FILE* file) {
    if (!valid_ || file == nullptr) return false;

    float* buf = new (std::nothrow) float[static_cast<size_t>(weight_count_)];
    if (buf == nullptr) return false;

    size_t n = std::fread(buf, sizeof(float), static_cast<size_t>(weight_count_), file);
    if (n != static_cast<size_t>(weight_count_)) {
        delete[] buf;
        return false;
    }

    delete[] weights_;
    weights_ = buf;
    return true;
}

float NeuralNetwork::activate(float x) const {
    switch (activation_) {
        case Activation::TANH:       return std::tanh(x);
        case Activation::RELU:       return (x > 0.0f) ? x : 0.0f;
        case Activation::LEAKY_RELU: return (x > 0.0f) ? x : leaky_slope_ * x;
        case Activation::IDENTITY:   return x;
    }
    return x;
}

bool NeuralNetwork::predict(const float* inputs, float* outputs) const {
    const int out_n = outputSize();
    if (outputs != nullptr) {
        for (int i = 0; i < out_n; ++i) outputs[i] = 0.0f;
    }
    if (!ready() || inputs == nullptr || outputs == nullptr) return false;

    float* cur = scratch_a_;
    float* nxt = scratch_b_;
    for (int i = 0; i < sizes_[0]; ++i) cur[i] = inputs[i];

    const float* cursor = weights_;
    for (int layer = 0; layer < num_layers_ - 1; ++layer) {
        const int in_n = sizes_[layer];
        const int layer_out = sizes_[layer + 1];
        const bool last = (layer == num_layers_ - 2);
        const float* w = cursor;
        const float* bias = cursor + (layer_out * in_n);

        for (int j = 0; j < layer_out; ++j) {
            float sum = bias[j];
            const float* row = w + (j * in_n);
            for (int i = 0; i < in_n; ++i) sum += row[i] * cur[i];
            nxt[j] = last ? sum : activate(sum);
        }

        cursor = bias + layer_out;
        float* swap = cur;
        cur = nxt;
        nxt = swap;
    }

    for (int j = 0; j < out_n; ++j) outputs[j] = cur[j];
    return true;
}

float NeuralNetwork::predictScalar(const float* inputs) const {
    if (outputSize() != 1) return 0.0f;
    float y = 0.0f;
    predict(inputs, &y);
    return y;
}

#pragma once
/* ---------------------------------------------------------------------------
 * NeuralNetwork  --  multilayer perceptron forward pass.
 *
 * Shared evaluator for learned models. NeuralFeedforward owns the chassis
 * topologies and calls this; nothing else needs to know how a layer is packed.
 * The network does not train. Hidden layers use the chosen activation and the
 * output layer stays linear, which is what a voltage regression needs.
 *
 * Weight layout, one layer at a time, input toward output:
 *   row-major weights[out][in], then biases[out]
 *   w[j * in + i] connects input i to output j (same packing as nn.Linear)
 * ------------------------------------------------------------------------- */

#include <cstdio>

class NeuralNetwork {
public:
    enum class Activation { IDENTITY, TANH, RELU, LEAKY_RELU, SILU };

    // Includes the input and output layers. Widest layer the scratch buffers hold.
    static constexpr int kMaxLayers = 8;
    static constexpr int kMaxUnits  = 256;

    // layer_sizes is copied. leaky_slope is the negative side of LEAKY_RELU (0.01
    // matches the usual PyTorch default).
    NeuralNetwork(const int* layer_sizes, int num_layers, Activation activation,
                  float leaky_slope = 0.01f);
    ~NeuralNetwork();

    NeuralNetwork(const NeuralNetwork&) = delete;
    NeuralNetwork& operator=(const NeuralNetwork&) = delete;

    // Floats in one network: for each layer, (out * in) weights plus `out` biases.
    static constexpr int countWeights(const int* layer_sizes, int num_layers) {
        if (layer_sizes == nullptr || num_layers < 2) return 0;
        int count = 0;
        for (int layer = 0; layer < num_layers - 1; ++layer) {
            int in_n = layer_sizes[layer];
            int out_n = layer_sizes[layer + 1];
            if (in_n < 1 || out_n < 1) return 0;
            count += out_n * in_n + out_n;
        }
        return count;
    }

    int inputSize() const  { return valid_ ? sizes_[0] : 0; }
    int outputSize() const { return valid_ ? sizes_[num_layers_ - 1] : 0; }
    int weightCount() const { return weight_count_; }
    bool ready() const { return valid_ && weights_ != nullptr; }

    // Copy `count` floats. count must equal weightCount().
    bool assignWeights(const float* weights, int count);

    // Read the next weightCount() floats from the current file position.
    bool loadWeights(FILE* file);

    // inputs has inputSize() values. outputs has outputSize() values.
    // Unloaded networks write zeros and return false.
    // Not safe to call predict() on the same instance from two tasks at once.
    bool predict(const float* inputs, float* outputs) const;

    // Single-output models (the chassis voltage heads). Returns 0 if the
    // network is not a scalar output or has no weights yet.
    float predictScalar(const float* inputs) const;

private:
    float activate(float x) const;

    int sizes_[kMaxLayers];
    int num_layers_;
    Activation activation_;
    float leaky_slope_;
    int weight_count_;
    float* weights_;
    bool valid_;

    mutable float scratch_a_[kMaxUnits];
    mutable float scratch_b_[kMaxUnits];
};

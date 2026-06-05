#pragma once

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>
#include <algorithm>
#include <limits.h>
#include <random>
#include <cmath>

static constexpr int FEAT_DIM = 1280;
static constexpr int REC_DIM  = 1281;

struct Event
{
    std::vector<float> features;  // (1280,)
    float joy_value;
};

struct Episode
{
    std::vector <Event> events;
    int size() const {return (int)events.size();}
};

struct Particle
{
    int episode_idx;
    int event_idx;
    double weight;
};

class ParticleFilter{
public:
    ParticleFilter();
    int init(const std::string& data_dir);
    void cycle(const std::vector<float>& feat);
    float decision() const;
private:
    int particle_num;
    std::mt19937 rng_;

    std::vector<Particle> particles_;
    std::vector<Episode>  episodes_;

    int load_episode(const std::string& path);
    void scatterParticles();
    void predict();
    void weighting(const std::vector<float>& feat);
    void normalizeWeights();
    void resampling();
    double likelihood(const std::vector<float>& a,
                    const std::vector<float>& b) const;
    double sumWeight() const;
};
#include "pfoe_localization/ParticleFilter.hpp"

ParticleFilter::ParticleFilter() : particle_num(1000),rng_(42)
{

}

int ParticleFilter::init(const std::string& data_dir){
    episodes_.clear();

    std::vector<std::string> paths;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(data_dir)) {
        if (entry.is_regular_file() &&
            entry.path().extension() == ".bin") {   // data.bin が引っかかる
            paths.push_back(entry.path().string());
        }
    }
    if (paths.empty()) {
        std::cerr << "no .bin under: " << data_dir << std::endl;
        return -1;                                  // 1個も無ければ失敗
    }
}

int ParticleFilter::load_episode(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return -1;

    std::streamsize bytes = f.tellg();
    const std::streamsize rec_bytes = REC_DIM * sizeof(float);
    if (bytes <= 0 || bytes % rec_bytes != 0) {
        std::cerr << "bad size: " << path << " (" << bytes << "B)" << std::endl;
        return -1;
    }
    int n_records = static_cast<int>(bytes / rec_bytes);

    f.seekg(0, std::ios::beg);
    Episode ep;
    ep.events.reserve(n_records);

    std::vector<float> rec(REC_DIM);
    while (f.read(reinterpret_cast<char*>(rec.data()), rec_bytes)) {
        Event e;
        e.features.assign(rec.begin(), rec.begin() + FEAT_DIM);
        e.joy_value = rec[FEAT_DIM];
        ep.events.push_back(std::move(e));
    }

    episodes_.push_back(std::move(ep));
    return 0;
}

void ParticleFilter::scatterParticles()
{
    particles_.resize(particle_num);

    std::vector<int> sizes;
    for (const auto& ep : episodes_) sizes.push_back(ep.size());
    std::discrete_distribution<int> ep_dist(sizes.begin(), sizes.end());

    double w = 1.0 / particle_num;
    for (auto& p : particles_) {
        p.episode_idx = ep_dist(rng_);
        std::uniform_int_distribution<int> ev_dist(
            0, episodes_[p.episode_idx].size() - 1);
        p.event_idx = ev_dist(rng_);
        p.weight = w;
    }
}


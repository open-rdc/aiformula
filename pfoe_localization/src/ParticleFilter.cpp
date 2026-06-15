#include "pfoe_localization/ParticleFilter.hpp"

ParticleFilter::ParticleFilter() : particle_num_(1000),rng_(42)
{}

int ParticleFilter::init(const std::string& data_dir, int predictio_range){
    episodes_.clear();

    prediction_range_ = predictio_range;
    if(predictio_range < 1){
        std::cerr << "invalid prediction_range: " << predictio_range << std::endl;
        return -1; 
    }

    std::vector<std::string> paths;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(data_dir)) {
        if (entry.is_regular_file() &&
            entry.path().extension() == ".bin") {
            paths.push_back(entry.path().string());
        }
    }

    if (paths.empty()){
        std::cerr << "no .bin under: " << data_dir << std::endl;
        return -1;
    }

    std::sort(paths.begin(), paths.end());

    for (const auto& p : paths) {
        if (loadEpisode(p) != 0){
            std::cerr << "failed to load: " << p << std::endl;
            return -1;
        }
    }
    
    scatterParticles();

    return 0;
}

void ParticleFilter::cycle(const std::vector<float>& feat){
    predict();

    for(auto &p : particles_){
        const auto& recorded = episodes_[p.episode_idx].events[p.event_idx].features;
        double h = likelihood(feat, recorded);

        p.weight *= (h + 1) / 2;
    }

    normalize();
    resampling();
}

float ParticleFilter::decision(){
    /*
    std::map<std::pair<int,int>, int> votes;

    for (const auto& p : particles_) {
        votes[{p.episode_idx, p.event_idx}]++;
    }

    std::pair<int,int> best;
    std::vector<std::pair<int,int>> best_votes;
    int max_count = -1;

    for (const auto& [cell, count] : votes) {
        if (count > max_count){
            max_count = count;
            best_votes.clear();
            best_votes.push_back(cell);
        }else if(count == max_count){
            best_votes.push_back(cell);
        }
    }
    */

    double sum = 0.0;
    double sum_w = 0.0;

    for (const auto& p : particles_) {
        float steer = episodes_[p.episode_idx].events[p.event_idx].joy_value;
        sum += p.weight * steer;
        sum_w += p.weight;
    }

    if (sum_w == 0.0) return 0.0f;

    return sum / sum_w;

}

int ParticleFilter::loadEpisode(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return -1;

    std::streamsize bytes = f.tellg();
    const std::streamsize rec_bytes = REC_DIM * sizeof(float);
    if (bytes <= 0 || bytes % rec_bytes != 0) return -1;
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
    particles_.resize(particle_num_);

    std::vector<int> sizes;
    for (const auto& ep : episodes_) sizes.push_back(ep.size());
    std::discrete_distribution<int> ep_dist(sizes.begin(), sizes.end());

    double w = 1.0 / particle_num_;
    for (auto& p : particles_) {
        p.episode_idx = ep_dist(rng_);
        std::uniform_int_distribution<int> ev_dist(
            0, episodes_[p.episode_idx].size() - 1);
        p.event_idx = ev_dist(rng_);
        p.weight = w;
    }
}

void ParticleFilter::predict(){
    for(auto &p : particles_){
        bool teleport = rand() % 10 == 0;
        if(!teleport){
            p.event_idx += rand() % prediction_range_;
            teleport = (p.event_idx >= (int)episodes_[p.episode_idx].size());
        }
        if(teleport){
            p.episode_idx = randomInt(0, episodes_.size()-1);
            int last = episodes_[p.episode_idx].size() - 1;
            p.event_idx = randomInt(0, last);
        }
    }
}

double ParticleFilter::likelihood(const std::vector<float>& a, const std::vector<float>& b) const{
    double dot_product = 0.0;
    double sum_sq_a = 0.0;
    double sum_sq_b = 0.0;

    for(int i = 0; i < FEAT_DIM; i++){
        dot_product += a[i] * b[i];
        sum_sq_a += a[i] * a[i];
        sum_sq_b += b[i] * b[i];
    }

    double denom = std::sqrt(sum_sq_a) * std::sqrt(sum_sq_b);
    return denom > 0.0 ? dot_product / denom : 0.0;
}

void ParticleFilter::normalize(){
    double sum = sumWeight();
    for(auto &p : particles_){
        p.weight /= sum;
    }
}

void ParticleFilter::resampling(){
    std::vector<Particle> prev;
    std::vector<Particle> next;

    std::shuffle(particles_.begin(), particles_.end(), rng_);

    double sum = 0.0;
    int j = 0;
    int num = (int)particles_.size();

    for(auto &p : particles_){
        p.weight += sum;
        sum = p.weight;
        prev.push_back(p);
    }

    double step = sum / num;
    double accum = step * randomReal(0.0, 1.0);

    for(int i = 0; i < num; i++){
        while(j < num-1 && prev[j].weight < accum) j++;
        next.push_back(prev[j]);
        accum += step;
    }

    for(auto &p : next) p.weight = 1.0 / num;
    particles_ = std::move(next);
}

int ParticleFilter::randomInt(int lo, int hi) {
    return std::uniform_int_distribution<int>(lo, hi)(rng_);
}

double ParticleFilter::randomReal(double lo, double hi) {
    return std::uniform_real_distribution<double>(lo, hi)(rng_);
}

double ParticleFilter::sumWeight() const{
    double sum = 0.0;
    for(const auto &p : particles_){
        sum += p.weight;
    }

    return sum;
}
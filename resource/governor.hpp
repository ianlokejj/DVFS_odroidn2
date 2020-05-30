#ifndef GOVERNOR_HPP
#define GOVERNOR_HPP

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <iostream>
#include <sched.h>
#include <atomic>
#include <chrono>
#include <numeric>
#include <thread>
#include <iterator>
#include <algorithm>
#include <utility>
#include <map>

#include "types.h"

#define SAMPLE_INTERVAL 50
#define SAMPLE_TIME 1000
#define SAMPLE_SIZE (SAMPLE_TIME / SAMPLE_INTERVAL)
#define THRESHOLD 2

#define LIMIT(x) (x == SMALL_CLUSTER ? 0.3 : 0.5)

#define SMALL_CLUSTER 0
#define BIG_CLUSTER 1
#define CLUSTER(x) (x == SMALL_CLUSTER ? CPU::SmallCores : CPU::BigCores)
#define CLUSTER_TYPE(x) (x == SMALL_CLUSTER ? CoreType::SMALL : CoreType::BIG)
#define CLUSTER_MIN_INDEX(x) (x == SMALL_CLUSTER ? CPU::s_range.first : CPU::b_range.first)
#define CLUSTER_MAX_INDEX(x) (x == SMALL_CLUSTER ? CPU::s_range.second : CPU::b_range.second)

enum State
{
    MONITOR,
    DECREASE_FREQUENCY,
    INCREASE_FREQUENCY,
    DECREASE_CORE,
    INCREASE_CORE,
    SET_PROFILE_RESULT,
    TO_BIG_CLUSTER
};

class Governor
{
public:
    Governor();
    ~Governor();

    void set_target_fps(double fps);
    double get_target_fps();
    double get_cur_fps();
    int get_sample_interval();
    int get_sample_time();

    const std::string get_governor(CoreType core_type);
    void set_governor(CoreType core_type, const std::string& governor);

    const std::string get_cur_freq(CoreType core_type);
    const std::string get_scaling_freq(CoreType core_type);
    void set_freq(CoreType core_type, const std::string& freq);

    void run_on_cores(const Core& core, const int pid = 0);
    Core runs_on(const int pid = 0);

    void start(const std::vector<int>& pids);
    void stop();
    void worked(int num_frames, double time_elapsed_ms);

private:
    std::atomic<double> fps; // FPS to dispaly, an running average
    std::atomic<double> instance_fps;
    std::atomic<double> target_fps;
    double running_fps;
    std::string s_governor;
    std::string b_governor;
    std::string s_scaling_min_freq;
    std::string b_scaling_min_freq;

    double sampled_fps[SAMPLE_SIZE] = {0};
    int curr_sample_index;
    int prev_sample_index;
    std::vector<pid_t> pids;
    std::vector<std::string> freqs[2];
    int pos;
    int prev_pos;
    int core_index;
    State state;

    int count;
    int cluster;
    std::thread* monitor_thread;
    bool ended;
    double prev_fps;
    std::map<int, std::map<int, double, std::greater<>>> profile_results[2];
    std::pair<int, int> config_pair;
    void search_profile_result();
    void set_profile_result();
    void monitor();
    void decrease_frequency();
    void increase_frequency();
    void increase_core();
    void decrease_core();
    void to_big_cluster();
    void profile();
    void next_state();
    void print_state();
    const char* to_string();
    bool exceed_threashold();

    const std::string get_policy_attribute(CoreType core_type, const std::string& attribute);
    void set_policy_attribute(CoreType core_type, const std::string& attribute, const std::string& data);
};

void Governor::monitor()
{
    // Set to run on small cores
    run_on_cores(CPU::SmallCores, 0);
    // std::ofstream myfile;
    // myfile.open ("logger.csv");

    while(!ended)
    {
        // TODO: pointer to pids is not a good implementation
        std::this_thread::sleep_for(std::chrono::milliseconds(SAMPLE_INTERVAL));

        // 1st FSM: calcuate the FPS, triggering every SAMPLE_INTERVAL
        {
            // Calculate the average FPS
            sampled_fps[curr_sample_index] = instance_fps;
            running_fps = running_fps + (sampled_fps[curr_sample_index++] - sampled_fps[prev_sample_index++]) / SAMPLE_SIZE;
            curr_sample_index %= SAMPLE_SIZE;
            prev_sample_index %= SAMPLE_SIZE;

            if(curr_sample_index == 0)
            {
                fps = running_fps;
            }
        }

        // FSM triggering, every half of the SAMPLE_TIME
        if(curr_sample_index != 0 && curr_sample_index != (SAMPLE_SIZE / 2))
            continue;

        // 2nd FSM, profiler
        profile();

        // 3rd FSM
        switch(state)
        {
        case MONITOR:
            break;
        case DECREASE_FREQUENCY:
            decrease_frequency();
            break;
        case INCREASE_FREQUENCY:
            increase_frequency();
            break;
        case INCREASE_CORE:
            increase_core();
            break;
        case DECREASE_CORE:
            decrease_core();
            break;
        case SET_PROFILE_RESULT:
            set_profile_result();
            break;
        case TO_BIG_CLUSTER:
            to_big_cluster();
        }
        next_state();
       print_state();
        // myfile << running_fps << ",\n";
    }
}

void Governor::decrease_frequency()
{
    pos = std::max(pos - 1, 0);
    set_freq(CLUSTER_TYPE(cluster), freqs[cluster][pos]);
}

void Governor::increase_frequency()
{
    pos = std::min(pos + 1, (int)freqs[cluster].size() - 1);
    set_freq(CLUSTER_TYPE(cluster), freqs[cluster][pos]);
}

void Governor::decrease_core()
{
    // Check if number of CPU goes below the minimum number of CPU
    if(core_index == CLUSTER_MIN_INDEX(cluster))
    {
        return;
    }

    Core cores = runs_on(pids[0]);
    cores.less_core(core_index);
    core_index--;

    for(pid_t tid : pids)
    {
        run_on_cores(cores, tid);
    }
}

void Governor::increase_core()
{
    // Check if number of CPU exceeds maximum CPU available
    // TODO: improve this
    if(core_index == CLUSTER_MAX_INDEX(cluster))
    {
        return;
    }

    core_index++;
    Core cores = runs_on(pids[0]);
    cores.add_core(core_index);

    for(pid_t tid : pids)
    {
        run_on_cores(cores, tid);
    }
}

void Governor::to_big_cluster()
{
    // Medium frequency, highest core count, empirically determined
    pos = freqs[BIG_CLUSTER].size() / 2;
    core_index = CPU::num_cpu - 1;

    const auto& clusters = CLUSTER(BIG_CLUSTER);
    for(pid_t tid : pids)
    {
        run_on_cores(clusters, tid);
    }
    set_freq(CLUSTER_TYPE(BIG_CLUSTER), freqs[BIG_CLUSTER][pos]);
    cluster = BIG_CLUSTER;
}

void Governor::profile()
{
    profile_results[cluster][pos][core_index] = running_fps;
}

void Governor::next_state()
{
    switch(state)
    {
    case MONITOR:
    {
        if(!exceed_threashold())
        {
            break;
        }

        // If target FPS has changed, apply profiled result once
        if(target_fps != prev_fps)
        {
            prev_fps = target_fps;
            state = SET_PROFILE_RESULT;
            break;
        }

        // Exceeded threshold
        double diff = running_fps - target_fps;
        if(diff > LIMIT(cluster))
        {
            if(pos - 1 < 0)
            {
                state = DECREASE_CORE;
                prev_pos = -1; // Ensure no change of cores without trying the frequency options
                break;
            }

            // Record down the frequency setting before it is changed
            prev_pos = pos;
            state = DECREASE_FREQUENCY;
        }
        else if(diff < -LIMIT(cluster))
        {
            // Check if increasing the frequency goes back to the previous frequency setting
            if(pos + 1 == prev_pos)
            {
                // If so, decrease number of cores
                pos++;
                state = DECREASE_CORE;
                break;
            }

            // Check if increasing the frequncy exceeds core's maximum frequency settings
            if(pos + 1 == freqs[cluster].size())
            {
                // If so, increase number of cores
                state = INCREASE_CORE;

                // If current cluster is small cluster, migrate to big core
                if(cluster == SMALL_CLUSTER)
                {
                    state = TO_BIG_CLUSTER;
                    prev_fps = target_fps; // After migration, do not consult the profiled result
                }

                prev_pos = -1; // Ensure no change of cores without trying the frequency options
                break;
            }

            // Record down the frequency setting before it is changed
            prev_pos = pos;
            state = INCREASE_FREQUENCY;
        }
        else
        {
            state = MONITOR;
        }
    }
    break;
    default:
        state = MONITOR;
    }
}

void Governor::search_profile_result()
{
    double min = DBL_MAX;
    std::pair<int, int> min_pair;

    // Search profiled result that matches the target FPS
    // Since the profile map is ordered, it starts from minimum frequency and maximum number of cores as this gives minimum power
    // If configuration is within the limit, it returns immediately. If not, the configuraiton with closest FPS is returned
    for(const int cluster :
{
    SMALL_CLUSTER, BIG_CLUSTER
})
    {
        // Check if target FPS is below the maximum achievable FPS of the small cluster.
        // If so, do not search big cluster, use the small cluster result, this priorities using small cluster.
        // Small cluster is guaranteed to have at least one result as the system is started in the maximum performance configuration
        // of the small cluster
        if(cluster == BIG_CLUSTER && target_fps <= profile_results[SMALL_CLUSTER][freqs[SMALL_CLUSTER].size() - 1][CPU::s_range.second])
        {
            config_pair = min_pair;
            return;
        }

        for(const auto& freq_entry : profile_results[cluster])
        {
            for(const auto& index_entry : freq_entry.second)
            {
                double diff = std::abs(index_entry.second - target_fps);
                if(diff < LIMIT(cluster))
                {
                    config_pair = std::make_pair(freq_entry.first, index_entry.first);
                    this->cluster = cluster;
                    return;
                }

                if(diff < min)
                {
                    min_pair.first = freq_entry.first;
                    min_pair.second = index_entry.first;
                    this->cluster = cluster;
                    min = diff;
                }
            }
        }
    }

    config_pair = min_pair;
}

void Governor::set_profile_result()
{
    // Search profiled result
    search_profile_result();

    // Set CPU affinity
    if(config_pair.second != core_index)
    {
        Core core(-1);

        for(int i = CLUSTER_MIN_INDEX(cluster); i <= config_pair.second; i++)
        {
            core.add_core(i);
        }

        for(pid_t tid : pids)
            run_on_cores(core, tid);

        core_index = config_pair.second;
        prev_pos = -1; // Ensure no change of cores without trying the frequency options
    }

    // Set frequency
    if(config_pair.first != pos)
    {
        set_freq(CLUSTER_TYPE(cluster), freqs[cluster][config_pair.first]);
        pos = config_pair.first;
        prev_pos = -1; // Ensure no change of cores without trying the frequency options
    }
}

bool Governor::exceed_threashold()
{
    if(target_fps == -1)
    {
        return false;
    }

    double diff = running_fps - target_fps;
    if(diff > LIMIT(cluster) || diff < -LIMIT(cluster))
        count++;

    if(count < THRESHOLD)
        return false;

    count = 0;
    return true;
}

void Governor::print_state()
{
    std::cout << running_fps << std::endl;
}

const char* Governor::to_string()
{
    switch(state)
    {
    case MONITOR:
        return "MONITOR";
    case DECREASE_FREQUENCY:
        return "DECREASE_FREQUENCY";
    case INCREASE_FREQUENCY:
        return "INCREASE_FREQUENCY";
    case DECREASE_CORE:
        return "DECREASE_CORE";
    case INCREASE_CORE:
        return"INCREASE_CORE";
    case SET_PROFILE_RESULT:
        return "SET_PROFILE_RESULT";
    case TO_BIG_CLUSTER:
        return "TO_BIG_CLUSTER";
    default:
        return "UNKNOWN_STATE";
    }
}

void Governor::start(const std::vector<int>& pids)
{
    ended = false;
    this->pids.clear();
    std::copy(pids.begin(), pids.end(), std::back_inserter(this->pids));

    freqs[SMALL_CLUSTER].clear();
    freqs[SMALL_CLUSTER].push_back(ScalingFrequency::_100000KHz);
    freqs[SMALL_CLUSTER].push_back(ScalingFrequency::_250000KHz);
    freqs[SMALL_CLUSTER].push_back(ScalingFrequency::_500000KHz);
    freqs[SMALL_CLUSTER].push_back(ScalingFrequency::_667000KHz);
    freqs[SMALL_CLUSTER].push_back(ScalingFrequency::_1000000KHz);
    freqs[SMALL_CLUSTER].push_back(ScalingFrequency::_1200000KHz);
    freqs[SMALL_CLUSTER].push_back(ScalingFrequency::_1398000KHz);
    freqs[SMALL_CLUSTER].push_back(ScalingFrequency::_1512000KHz);
    freqs[SMALL_CLUSTER].push_back(ScalingFrequency::_1608000KHz);
    freqs[SMALL_CLUSTER].push_back(ScalingFrequency::_1704000KHz);
    freqs[SMALL_CLUSTER].push_back(ScalingFrequency::_1896000KHz);

    freqs[BIG_CLUSTER].clear();
    freqs[BIG_CLUSTER].push_back(ScalingFrequency::_100000KHz);
    freqs[BIG_CLUSTER].push_back(ScalingFrequency::_250000KHz);
    freqs[BIG_CLUSTER].push_back(ScalingFrequency::_500000KHz);
    freqs[BIG_CLUSTER].push_back(ScalingFrequency::_667000KHz);
    freqs[BIG_CLUSTER].push_back(ScalingFrequency::_1000000KHz);
    freqs[BIG_CLUSTER].push_back(ScalingFrequency::_1200000KHz);
    freqs[BIG_CLUSTER].push_back(ScalingFrequency::_1398000KHz);
    freqs[BIG_CLUSTER].push_back(ScalingFrequency::_1512000KHz);
    freqs[BIG_CLUSTER].push_back(ScalingFrequency::_1608000KHz);
    freqs[BIG_CLUSTER].push_back(ScalingFrequency::_1704000KHz);
    freqs[BIG_CLUSTER].push_back(ScalingFrequency::_1800000KHz);

    cluster = SMALL_CLUSTER; // Set to run on small cores
    pos = freqs[cluster].size() - 1; // Set to run on highest frequency
    prev_pos = -1;
    target_fps = -1;
    prev_fps = -1;
    state = MONITOR;

    for(pid_t tid : pids)
    {
        run_on_cores(CLUSTER(cluster), tid);
    }

    cpu_set_t set = runs_on(pids[0]);
    for(int i = 0; i < CPU::num_cpu; i++)
    {
        if(CPU_ISSET(i, &set))
            core_index = i;
    }

    // Set frequency
    set_freq(CLUSTER_TYPE(cluster), freqs[cluster][pos]);

    // Start the monitor thread
    monitor_thread = new std::thread(&Governor::monitor, this);
}

void Governor::stop()
{
    ended = true;

    if(monitor_thread != nullptr)
    {
        monitor_thread->join();
        delete monitor_thread;
    }
}

Governor::Governor()
{
    curr_sample_index = 0;
    prev_sample_index = 1;
    ended = true;

    // Get the original governor
    s_governor = get_governor(CoreType::SMALL);
    b_governor = get_governor(CoreType::BIG);

    // Get the original minimum scaling frequencies
    s_scaling_min_freq = get_policy_attribute(CoreType::SMALL, "scaling_min_freq");
    b_scaling_min_freq = get_policy_attribute(CoreType::BIG, "scaling_min_freq");

    // Set minimum scaling frequencies to the lowest capable value of the CPUs
    set_policy_attribute(CoreType::SMALL, "scaling_min_freq", get_policy_attribute(CoreType::SMALL, "cpuinfo_min_freq"));
    set_policy_attribute(CoreType::BIG, "scaling_min_freq", get_policy_attribute(CoreType::BIG, "cpuinfo_min_freq"));

    // Set governor to userspace
    set_governor(CoreType::SMALL, CPUGovernor::userspace);
    set_governor(CoreType::BIG, CPUGovernor::userspace);

    // Set frequency to maximum
    set_freq(CoreType::SMALL, ScalingFrequency::_1896000KHz);
    set_freq(CoreType::BIG, ScalingFrequency::_1800000KHz);
}

Governor::~Governor()
{
    // Restore original governor
    set_governor(CoreType::SMALL, s_governor);
    set_governor(CoreType::BIG, b_governor);

    // Restore original minimum scaling frequencies
    set_policy_attribute(CoreType::SMALL, "scaling_min_freq", s_scaling_min_freq);
    set_policy_attribute(CoreType::BIG, "scaling_min_freq", b_scaling_min_freq);
}

void Governor::set_governor(CoreType core_type, const std::string& governor)
{
    set_policy_attribute(core_type, "scaling_governor", governor);
}

const std::string Governor::get_governor(CoreType core_type)
{
    return get_policy_attribute(core_type, "scaling_governor");
}

void Governor::set_freq(CoreType core_type, const std::string& freq)
{
    set_policy_attribute(core_type, "scaling_setspeed", freq);
}

const std::string Governor::get_cur_freq(CoreType core_type)
{
    return get_policy_attribute(core_type, "cpuinfo_cur_freq");
}

const std::string Governor::get_scaling_freq(CoreType core_type)
{
    return get_policy_attribute(core_type, "scaling_cur_freq");
}

int Governor::get_sample_interval()
{
    return SAMPLE_INTERVAL;
}

int Governor::get_sample_time()
{
    return SAMPLE_TIME;
}

void Governor::run_on_cores(const Core& core, const int pid)
{
    const cpu_set_t& mask = static_cast<cpu_set_t>(core);
    if (sched_setaffinity(pid, sizeof(cpu_set_t), &mask) == -1)
    {
        std::cerr << "PID " << pid << " fail to set CPU affinity" << std::endl;
    }
}

Core Governor::runs_on(const int pid)
{
    cpu_set_t mask;
    if(sched_getaffinity(pid, sizeof(mask), &mask) == -1)
    {
        std::cerr << "PID " << pid << " fail to get CPU affinity" << std::endl;
    }

    return Core(mask);
}

void Governor::set_target_fps(double fps)
{
    prev_fps = target_fps;
    target_fps = fps;
}

double Governor::get_target_fps()
{
    return target_fps;
}

double Governor::get_cur_fps()
{
    return fps.load();
}

void Governor::worked(int num_frames, double time_elapsed_ms)
{
    instance_fps = num_frames / (time_elapsed_ms / 1000.0);
}

const std::string Governor::get_policy_attribute(CoreType core_type, const std::string& attribute)
{
    std::ifstream file;
    if (core_type == SMALL)
    {
        file.open("/sys/devices/system/cpu/cpufreq/policy0/" + attribute);
    }
    else
    {
        file.open("/sys/devices/system/cpu/cpufreq/policy2/" + attribute);
    }

    std::string data;
    file >> data;

    file.close();

    return data;
}

void Governor::set_policy_attribute(CoreType core_type, const std::string& attribute, const std::string& data)
{
    std::ofstream file;
    if (core_type == SMALL)
    {
        file.open("/sys/devices/system/cpu/cpufreq/policy0/" + attribute);
    }
    else
    {
        file.open("/sys/devices/system/cpu/cpufreq/policy2/" + attribute);
    }

    file << data;

    file.close();
}

#endif // GOVERNOR_HPP

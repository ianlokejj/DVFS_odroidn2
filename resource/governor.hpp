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

#include "types.h"

class Governor
{
public:
    Governor();
    ~Governor();

    int set_target_fps(double fps, const std::vector<int>& pids);
    int set_core_and_freq(int position, const std::vector<int>& pids);
    double get_target_fps();
    double get_cur_fps();

    const std::string get_governor(CoreType core_type);
    void set_governor(CoreType core_type, const std::string& governor);

    const std::string get_cur_freq(CoreType core_type);
    const std::string get_scaling_freq(CoreType core_type);
    void set_freq(CoreType core_type, const std::string& freq);

    void run_on_cores(const Core& core, const int pid = 0);
    const Core runs_on(const int pin = 0);

    void worked(int num_frames, double time_elapsed_ms);

private:
    std::atomic<double> fps;
    double target_fps = 0.0;
    std::string s_governor;
    std::string b_governor;
    std::string s_scaling_min_freq;
    std::string b_scaling_min_freq;

    const std::string get_policy_attribute(CoreType core_type, const std::string& attribute);
    void set_policy_attribute(CoreType core_type, const std::string& attribute, const std::string& data);
};

Governor::Governor()
{
    fps = 0.0;

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

    // Set to run on all cores
    run_on_cores(CPU::AllCores);
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

void Governor::run_on_cores(const Core& core, const int pid)
{
    const cpu_set_t& mask = static_cast<cpu_set_t>(core);
    if (sched_setaffinity(pid, sizeof(mask), &mask) == -1)
    {
        std::cerr << "PID " << pid << " fail to set CPU affinity" << std::endl;
    }
}

const Core Governor::runs_on(const int pid)
{
    cpu_set_t mask;
    if(sched_getaffinity(pid, sizeof(mask), &mask) == -1)
    {
        std::cerr << "PID " << pid << " fail to get CPU affinity" << std::endl;
    }

    int count = 0;
    for(int i = 2; i<=5; i++)
    {
        if(CPU_ISSET(i, &mask) == 1){
            count++;
        }
    }
    cv::setNumThreads(count);

    return Core(mask);
}

int Governor::set_target_fps(double fps, const std::vector<int>& pids)
{
    double average_fps;
    std::cout << "set_target_fps called" << std::endl;
    std::chrono::seconds SampleTime = std::chrono::seconds(30)/10;
    auto last_time = std::chrono::system_clock::now();

    double error;
    int position = 20;
    std::vector<double> v;

    while(1){
      auto now = std::chrono::system_clock::now();
      v.push_back(get_cur_fps());
      // add current fps to vector
      auto timeChange = (now - last_time);
      if(timeChange >= SampleTime)
      {
        // take the average of current fps
        average_fps = std::accumulate( v.begin(), v.end(), 0.0)/v.size();
        std::cout << "average_fps: " << average_fps << std::endl;

        error = fps - average_fps;

        // clear the fps_vector
        v.clear();

        if (error > 1)
        {
          // increase computing power
          position += 1;
          set_core_and_freq(position, pids);
        }
        else if (error < -1)
        {
          // reduce computing power
          position -= 1;
          set_core_and_freq(position, pids);
        }

        std::cout << "position: " << position << std::endl;
        last_time = now;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return fps;
}

int Governor::set_core_and_freq(int position, const std::vector<int>& pids)
{
  Core cores;
  CoreType core_type;
  std::string freq;

  // make sure position is in range
  if (position > 20)
  {
    std::cout << "Already at 20!" << std::endl;
    return 20;
  }
  else if (position < 0)
  {
    std::cout << "Already at 0!" << std::endl;
    return 0;
  }

  switch(position) {
      case 0 :
        // A73_100000KHz
        cores = CPU::BigCores;
        core_type = CoreType::BIG;
        freq = ScalingFrequency::_100000KHz;
        std::cout << "A73_100000KHz" << std::endl;
        break;
      case 1 :
        // A53_250000KHz
        cores = CPU::SmallCores;
        core_type = CoreType::SMALL;
        freq = ScalingFrequency::_250000KHz;
        std::cout << "A53_250000KHz" << std::endl;
        break;
      case 2:
        // A73_250000KHz
        cores = CPU::BigCores;
        core_type = CoreType::BIG;
        freq = ScalingFrequency::_250000KHz;
        std::cout << "A73_250000KHz" << std::endl;
        break;
      case 3:
        // A53_500000KHz
        cores = CPU::SmallCores;
        core_type = CoreType::SMALL;
        freq = ScalingFrequency::_500000KHz;
        std::cout << "A53_500000KHz" << std::endl;
        break;
      case 4:
        // A53_667000KHz
        cores = CPU::SmallCores;
        core_type = CoreType::SMALL;
        freq = ScalingFrequency::_667000KHz;
        std::cout << "A53_667000KHz" << std::endl;
        break;
      case 5:
        // A53_1000000KHz
        cores = CPU::SmallCores;
        core_type = CoreType::SMALL;
        freq = ScalingFrequency::_1000000KHz;
        std::cout << "A53_1000000KHz" << std::endl;
        break;
      case 6:
        // A53_1200000KHz
        cores = CPU::SmallCores;
        core_type = CoreType::SMALL;
        freq = ScalingFrequency::_1200000KHz;
        std::cout << "A53_1200000KHz" << std::endl;
        break;
      case 7:
        // A53_1398000KHz
        cores = CPU::SmallCores;
        core_type = CoreType::SMALL;
        freq = ScalingFrequency::_1398000KHz;
        std::cout << "A53_1398000KHz" << std::endl;
        break;
      case 8:
        // A73_500000KHz
        cores = CPU::BigCores;
        core_type = CoreType::BIG;
        freq = ScalingFrequency::_667000KHz;
        std::cout << "A73_500000KHz" << std::endl;
        break;
      case 9:
        // A53_1512000KHz
        cores = CPU::SmallCores;
        core_type = CoreType::SMALL;
        freq = ScalingFrequency::_1512000KHz;
        std::cout << "A53_1512000KHz" << std::endl;
        break;
      case 10:
        // A53_1608000KHz
        cores = CPU::SmallCores;
        core_type = CoreType::SMALL;
        freq = ScalingFrequency::_1608000KHz;
        std::cout << "A53_1608000KHz" << std::endl;
        break;
      case 11:
        // A53_1704000KHz
        cores = CPU::SmallCores;
        core_type = CoreType::SMALL;
        freq = ScalingFrequency::_1704000KHz;
        std::cout << "A53_1704000KHz" << std::endl;
        break;
      case 12:
        // A53_1896000KHz
        cores = CPU::SmallCores;
        core_type = CoreType::SMALL;
        freq = ScalingFrequency::_1896000KHz;
        std::cout << "A53_1896000KHz" << std::endl;
        break;
      case 13:
        // A73_667000KHz
        cores = CPU::BigCores;
        core_type = CoreType::BIG;
        freq = ScalingFrequency::_667000KHz;
        std::cout << "A73_667000KHz" << std::endl;
        break;
      case 14:
        // A73_1000000KHz
        core_type = CoreType::BIG;
        cores = CPU::BigCores;
        freq = ScalingFrequency::_1000000KHz;
        std::cout << "A73_1000000KHz" << std::endl;
        break;
      case 15:
        // A73_1200000KHz
        cores = CPU::BigCores;
        core_type = CoreType::BIG;
        freq = ScalingFrequency::_1200000KHz;
        std::cout << "A73_1200000KHz" << std::endl;
        break;
      case 16:
        // A73_1398000KHz
        cores = CPU::BigCores;
        core_type = CoreType::BIG;
        freq = ScalingFrequency::_1398000KHz;
        std::cout << "A73_1398000KHz" << std::endl;
        break;
      case 17:
        // A73_1512000KHz
        cores = CPU::BigCores;
        core_type = CoreType::BIG;
        freq = ScalingFrequency::_1512000KHz;
        std::cout << "A73_1512000KHz" << std::endl;
        break;
      case 18:
        // A73_1608000KHz
        cores = CPU::BigCores;
        core_type = CoreType::BIG;
        freq = ScalingFrequency::_1608000KHz;
        std::cout << "A73_1608000KHz" << std::endl;
        break;
      case 19:
        // A73_1704000KHz
        cores = CPU::BigCores;
        core_type = CoreType::BIG;
        freq = ScalingFrequency::_1704000KHz;
        std::cout << "A73_1704000KHz" << std::endl;
        break;
      case 20:
        // A73_1800000KHz
        cores = CPU::BigCores;
        core_type = CoreType::BIG;
        freq = ScalingFrequency::_1800000KHz;
        std::cout << "A73_1800000KHz" << std::endl;
        break;
  }

  for(int pid : pids)
  {
      run_on_cores(cores, pid);
  }
  set_freq(core_type, freq);
  return position;
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
    fps = num_frames / (time_elapsed_ms / 1000.0);
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

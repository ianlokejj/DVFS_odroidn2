#ifndef TYPES_H
#define TYPES_H

/* Scaling frequencies, global context */
namespace ScalingFrequency
{
    extern const std::string _100000KHz;
    extern const std::string _250000KHz;
    extern const std::string _500000KHz;
    extern const std::string _667000KHz;
    extern const std::string _1000000KHz;
    extern const std::string _1200000KHz;
    extern const std::string _1398000KHz;
    extern const std::string _1512000KHz;
    extern const std::string _1608000KHz;
    extern const std::string _1704000KHz;
    extern const std::string _1800000KHz; // For big cores only
    extern const std::string _1896000KHz; // For small cores only
}

/* CPU governors, global context */
namespace CPUGovernor
{
    extern const std::string ondemand;
    extern const std::string powersave;
    extern const std::string userspace;
    extern const std::string conservative;
    extern const std::string interactive;
    extern const std::string performance;
    extern const std::string schedutil;
}

enum CoreType
{
    SMALL,
    BIG
};

class Core
{
public:
    Core();
    Core(int mask);
    Core(cpu_set_t mask);

    Core operator& (const Core& that);
    Core operator| (const Core& that);
    Core operator|= (const int cpu);
    bool operator== (const Core& that);
    void add_core(const int cpu);
    void less_core(const int cpu);
    int get_num_cores();
    operator cpu_set_t() const;
    operator std::string() const;

private:
    cpu_set_t mask;
};

/* CPU cores, global context */
namespace CPU
{
    extern const int num_cpu;
    extern const std::pair<int, int> s_range;
    extern const std::pair<int, int> b_range;
    extern Core Core0, Core1, Core2, Core3, Core4, Core5, SmallCores, BigCores, AllCores;
}

#endif // TYPES_H

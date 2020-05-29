#include <sched.h>
#include <string>

#include "types.h"

using namespace CPU;

/* CPU scaling frequency types */
namespace ScalingFrequency
{
const std::string _100000KHz("100000");
const std::string _250000KHz("250000");
const std::string _500000KHz("500000");
const std::string _667000KHz("667000");
const std::string _1000000KHz("1000000");
const std::string _1200000KHz("1200000");
const std::string _1398000KHz("1398000");
const std::string _1512000KHz("1512000");
const std::string _1608000KHz("1608000");
const std::string _1704000KHz("1704000");
const std::string _1800000KHz("1800000"); // For big cores only
const std::string _1896000KHz("1896000"); // For small cores only
}


/* CPU governor */
namespace CPUGovernor
{
const std::string ondemand("ondemand");
const std::string powersave("powersave");
const std::string userspace("userspace");
const std::string conservative("conservative");
const std::string interactive("interactive");
const std::string performance("performance");
const std::string schedutil("schedutil");
}

/* CPU core types */
Core::Core()
{
    CPU_ZERO(&mask);
}

Core::Core(int mask)
{
    CPU_SET(mask, &this->mask);
}

Core::Core(cpu_set_t mask)
{
    this->mask = mask;
}

Core Core::operator& (const Core& that)
{
    cpu_set_t new_mask;
    CPU_AND(&new_mask, &this->mask, &that.mask);
    return Core(new_mask);
}

Core Core::operator| (const Core& that)
{
    cpu_set_t new_mask;
    CPU_OR(&new_mask, &this->mask, &that.mask);
    return Core(new_mask);
}

Core Core::operator|= (const int cpu)
{
    CPU_SET(cpu, &mask);
}

bool Core::operator== (const Core& that)
{
    return CPU_EQUAL(&this->mask, &that.mask);
}

Core::operator cpu_set_t() const
{
    return mask;
}

Core::operator std::string() const
{
    std::string cpus;
    for(int i = 0; i < CPU::num_cpu; i++)
    {
        if(CPU_ISSET(i, &mask))
        {
            cpus = cpus + (cpus.empty() ? "" : " ") + std::to_string(i);
        }
    }

    return cpus;
}

namespace CPU
{
// Number of CPUs on this system
const int num_cpu = 6;

// Small cores
Core Core0(0);
Core Core1(1);

// Big cores
Core Core2(2);
Core Core3(3);
Core Core4(4);
Core Core5(5);

// Collection of cores
Core SmallCores(Core0 | Core1);
Core BigCores(Core2 | Core3 | Core4 | Core5);
Core AllCores(SmallCores | BigCores);
}

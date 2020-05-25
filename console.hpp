#ifndef CONSOLE_HPP
#define CONSOLE_HPP

#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>
#include <sys/syscall.h>

#include "types.h"

#define gettid() syscall(SYS_gettid)

class Console
{
public:
    void operator()(Governor& governor, std::function<void()> callback)
    {
        // Capture all opencv thread ids
        std::vector<int> pids;
        get_opencv_pids(pids);

        std::vector<std::string> freqs;
        freqs.push_back(ScalingFrequency::_100000KHz);
        freqs.push_back(ScalingFrequency::_250000KHz);
        freqs.push_back(ScalingFrequency::_500000KHz);
        freqs.push_back(ScalingFrequency::_667000KHz);
        freqs.push_back(ScalingFrequency::_1000000KHz);
        freqs.push_back(ScalingFrequency::_1200000KHz);
        freqs.push_back(ScalingFrequency::_1398000KHz);
        freqs.push_back(ScalingFrequency::_1512000KHz);
        freqs.push_back(ScalingFrequency::_1608000KHz);
        freqs.push_back(ScalingFrequency::_1704000KHz);
        freqs.push_back("");

        // Set to run on small cores
        governor.run_on_cores(CPU::SmallCores, 0);

        // Start the governor
        governor.start(pids);

        const std::string menu = "1. Set target FPS\n"
                                 "2. Set affinity\n"
                                 "3. Set scaling frequency\n"
                                 "4. Print application information\n"
                                 "5. Exist\n"
                                 "Input: ";

        while (1)
        {
            // Print menu
            std::cout << menu;

            // Wait for input
            std::string input;
            std::cin >> input;
            std::cout << std::endl;

            try
            {
                int i = std::stoi(input);
                switch (i)
                {
                case 1:
                {
                    std::cout << "Enter target fps: ";
                    std::cin >> input;

                    double fps = std::stod(input);
                    governor.set_target_fps(fps);
                    break;
                }
                case 2:
                    std::cout << "Enter \"big\", \"small\", \"all\" or mask(LSB-MSB): ";
                    std::cin >> input;
                    if (input == "small")
                    {
                        set_affinity(CPU::SmallCores, governor, pids);
                    }
                    else if(input == "big")
                    {
                        set_affinity(CPU::BigCores, governor, pids);
                    }
                    else if (input == "all")
                    {
                        set_affinity(CPU::AllCores, governor, pids);
                    }
                    else
                    {
                        try
                        {
                            Core cores;
                            for(int i = 0; i < CPU::num_cpu && i < input.length(); i++)
                            {
                                if(input.at(i) == '1')
                                {
                                    cores |= i;
                                }
                            }
                            set_affinity(cores, governor, pids);
                        }
                        catch(std::invalid_argument)
                        {
                            print_error_option();
                            break;
                        }
                    }

                    std::cout << "Application set to run on: " << governor.runs_on(pids[0]) << std::endl;
                    break;
                case 3:
                    std::cout << "Set [1=big, 2=small] cores: ";
                    std::cin >> input;
                    try
                    {
                        int choice = std::stoi(input);
                        switch(choice)
                        {
                        case 1:
                            freqs[10] = ScalingFrequency::_1800000KHz;
                        case 2:
                            freqs[10] = ScalingFrequency::_1896000KHz;

                            std::cout << "Choose one of the below:" <<std::endl;
                            for(int i = 0; i < freqs.size(); i++)
                            {
                                std::cout << std::to_string(i + 1) << "=" << freqs.at(i) << "KHz, ";
                            }
                            std::cout << std::endl << "Input: ";

                            std::cin >> input;
                            governor.set_freq(choice == 1 ? CoreType::BIG : CoreType::SMALL, freqs.at(std::stoi(input) - 1));
                            break;
                        default:
                            print_error_option();
                        }
                    }
                    catch(std::invalid_argument)
                    {
                        print_error_option();
                    }
                    break;
                case 4:
                    std::cout << "Target FPS: " << governor.get_target_fps() << std::endl;
                    std::cout << "Governer(big, small): " << governor.get_governor(CoreType::BIG) << ", " << governor.get_governor(CoreType::SMALL) << std::endl;
                    std::cout << "CPU Affinity: " << governor.runs_on(pids[0]) << std::endl;
                    std::cout << "Scaling Frequency(big, small): " << governor.get_scaling_freq(CoreType::BIG) << "KHz, " << governor.get_scaling_freq(CoreType::SMALL) << "KHz" << std::endl;
                    std::cout << "Operating Frquency(big, small): " << governor.get_cur_freq(CoreType::BIG) << "KHz, " << governor.get_cur_freq(CoreType::SMALL) << "KHz"<<  std::endl;
                    break;
                case 5:
                    callback();
                    return;
                default:
                    print_error_option();
                }
            }
            catch(const std::invalid_argument&)
            {
                print_error_option();
            }

            std::cout << std::endl;
        }
    }

private:
    void print_error_option()
    {
        std::cout << "Please enter a correct option." << std::endl;
    }

    void get_opencv_pids(std::vector<int>& pids)
    {
        system("ps -eL | grep DisplayImage > opencv_pids");

        std::ifstream fs("opencv_pids");
        while(1)
        {
            std::string results[5];
            for(int i = 0; i < 5; i++)
            {
                fs >> results[i];
            }

            if (fs.eof())
            {
                break;
            }
            pids.push_back(std::stoi(results[1]));
        }
        fs.close();

        // Excluse current tid of the IO thread and any other thread
        int tid = gettid();
        for(auto it = pids.begin(); it != pids.end(); it++)
        {
            if (*it == tid)
            {
                pids.erase(it);
                break;
            }
        }
    }

    void set_affinity(Core& cores, Governor& governor, const std::vector<int>& pids)
    {
        for(int pid : pids)
        {
            governor.run_on_cores(cores, pid);
        }
    }
};

#endif // CONSOLE_HPP

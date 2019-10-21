

/* Levels
4 - High frequent debug
3 - Debug
2 - Warnings
1 - Errors
0 - Critical
*/

#ifndef CustomLog_H
#define CustomLog_H

#include <iostream>
#include <fstream>
#include <chrono>
#include "ros/ros.h"
#include <ros/package.h>

#define LOG_LEVEL 3

#define DEBUG 3
#define INIT 2
#define WARNING 1
#define ERROR 0

#define INIT_COLOR "\033[0;32m"
#define ERROR_COLOR "\033[0;31m"
#define WARNING_COLOR "\033[0;33m"
#define END_ESC "\033[0m"

template <class T> 
void logMsg(const std::string& from_class, const T& msg, int level=3){

	if(level <= LOG_LEVEL){
		switch(level){
			case INIT:
				std::cout << INIT_COLOR << "[" << from_class << "]: " << msg << END_ESC << std::endl;
				break;

			case WARNING:
				std::cout << WARNING_COLOR << "[" << from_class << "]: " << msg << END_ESC << std::endl;
				break;

			case ERROR:
				std::cout << ERROR_COLOR << "[" << from_class << "]: " << msg << END_ESC << std::endl;
				break;

			default:
				std::cout << "[" << from_class << "]: " << msg << std::endl;
				break;
		}
		
	}
}


template <class T> 
void logTmp(const T& msg){

	std::cout << "Tmp Logging: \n" << msg << std::endl;
}

template <class T>
void logTmp(const std::string& name, const T& msg){

    std::cout << name + "\n" << msg << std::endl;
}

inline void logAssert(bool input, const std::string& message){

	if(input){
        
    }else{
        std::cout << WARNING_COLOR << "Assertion failed! : " << message << END_ESC << std::endl;
    }
}

void logMsg(const std::string& from_class, const std::string& msg, int level=3);


class RunCheck{

public:   
    RunCheck(std::string name_set){
        name = name_set;
        logTmp("Runcheck " + name + " started...");
    }
        
    ~RunCheck(){
        logTmp("Runcheck " + name + " completed!");
    }

private:

    std::string name;    
};


/** @brief Class for timing scopes. Constructor and destructor handle timing.
 * 
 * @param name[in] Optional name printed in the output */
class ScopeTimer{
public:
    ScopeTimer(){
        start_time = std::chrono::system_clock::now();
    }
    
    ScopeTimer(const std::string& name_){
        name = name_;
        start_time = std::chrono::system_clock::now();
    }
    
    virtual
    ~ScopeTimer(){
        auto end_time = std::chrono::system_clock::now();
        std::chrono::duration<double> duration_us = (end_time - start_time) * 1.0e6;
        logMsg("ScopeTimer", "[" + name + "] timed at " + std::to_string(duration_us.count()) + " us");
    }
    
protected:

    std::string name = "Unknown";
    std::chrono::system_clock::time_point start_time;

};


/** @brief Class for timing with file output */
class Benchmarker{
public:

    Benchmarker(const std::string& description_set, const std::string& filename_set){
        description = description_set;
        filename = ros::package::getPath("panda") + "/timing/" + filename_set + ".txt";
    }
    
    Benchmarker(){
        description = "Warning: Empty!";
    }
    
    virtual ~Benchmarker(){
        if(N_runs == 0){
            return;
        }
        
        std::ofstream timefile;
        try{
            
            // If a file exists, open it
            std::ifstream f(filename);
            if(f.good()){
                timefile.open(filename, std::ios_base::app);
                timefile << "\n";
            }else{
                
                // Otherwise, create it
                timefile.open (filename);
            }

            // Write results and close
            writeResults(timefile);
            timefile.close();
            
        }catch(std::exception ex){
            logMsg("Benchmarker", "Error loading " + filename + "!", 0);
        }
        
        
    }
    
    void start(){
        start_time = std::chrono::system_clock::now();        
    }
    
    void end(){
        auto end_time = std::chrono::system_clock::now();
        std::chrono::duration<double> duration_us = (end_time - start_time) * 1.0e6;
        double duration_double = duration_us.count();
        
        total_t += duration_double;
        N_runs++;
        
        if(duration_double > max_t){
            max_t = duration_double;
        }else if(duration_double < min_t){
            min_t = duration_double;
        }
    }
    
    void writeResults(std::ofstream& timefile){
        
        timefile << "Timing Results: " << description << "\n";
        timefile << "---------------------------------------\n";
        timefile << "Average T: " << total_t / (double)(N_runs) << "us \n";
        timefile << "Max T: " << max_t << "us \n";
        timefile << "Min T: " << min_t << "us \n";
    }

protected:
    std::string description = "";
    std::string filename = "";
    std::chrono::system_clock::time_point start_time;
    
    double max_t = 0.0;
    double min_t = 1e14;
    double total_t = 0.0;
    int N_runs = 0;
    
};



#endif
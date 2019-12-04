/**
* @brief Class for logging, debugging and profiling 
* /

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
#include <algorithm>
#include <thread>
#include <string>

#define LOG_LEVEL 3

#define DEBUG 3
#define INIT 2
#define WARNING 1
#define ERROR 0

#define INIT_COLOR "\033[0;32m"
#define ERROR_COLOR "\033[0;31m"
#define WARNING_COLOR "\033[0;33m"
#define END_ESC "\033[0m"

#define PROFILER 0
#if PROFILER
#define PROFILE_SCOPE(name) InstrumentationTimer timer##__LINE__(name)
#define PROFILE_FUNCTION() PROFILE_SCOPE(__FUNCTION__)
#else
#define PROFILE_SCOPE(name)
#define PROFILE_FUNCTION()
#endif

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


/** @brief Check if a scope is running */
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


// Basic instrumentation profiler by Cherno

// Usage: include this header file somewhere in your code (eg. precompiled header), and then use like:
//
// Instrumentor::Get().BeginSession("Session Name");   // Begin session 
// {
//     InstrumentationTimer timer("Profiled Scope Name");      // Place code like this in scopes you'd like to include in profiling
//     // Code
// }
// Instrumentor::Get().EndSession();                   // End Session
//
// You will probably want to macro-fy this, to switch on/off easily and use things like __FUNCSIG__ for the profile name.
//

struct ProfileResult
{
    std::string Name;
    long long Start, End;
    uint32_t ThreadID;
};

struct InstrumentationSession
{
    std::string Name;
};

class Instrumentor
{
private:
    InstrumentationSession* m_CurrentSession;
    std::ofstream m_OutputStream;
    int m_ProfileCount;
public:
    Instrumentor()
        : m_CurrentSession(nullptr), m_ProfileCount(0)
    {
    }

    void BeginSession(const std::string& name, const std::string& filepath = "results.json")
    {
        std::string full_filepath = ros::package::getPath("panda") + "/json/" + filepath;
        m_OutputStream.open(full_filepath);
        WriteHeader();
        m_CurrentSession = new InstrumentationSession{ name };
    }

    void EndSession()
    {
        WriteFooter();
        m_OutputStream.close();
        delete m_CurrentSession;
        m_CurrentSession = nullptr;
        m_ProfileCount = 0;
    }

    void WriteProfile(const ProfileResult& result)
    {
        if (m_ProfileCount++ > 0)
            m_OutputStream << ",";

        std::string name = result.Name;
        std::replace(name.begin(), name.end(), '"', '\'');

        m_OutputStream << "{";
        m_OutputStream << "\"cat\":\"function\",";
        m_OutputStream << "\"dur\":" << (result.End - result.Start) << ',';
        m_OutputStream << "\"name\":\"" << name << "\",";
        m_OutputStream << "\"ph\":\"X\",";
        m_OutputStream << "\"pid\":0,";
        m_OutputStream << "\"tid\":" << result.ThreadID << ",";
        m_OutputStream << "\"ts\":" << result.Start;
        m_OutputStream << "}";

        m_OutputStream.flush();
    }

    void WriteHeader()
    {
        m_OutputStream << "{\"otherData\": {},\"traceEvents\":[";
        m_OutputStream.flush();
    }

    void WriteFooter()
    {
        m_OutputStream << "]}";
        m_OutputStream.flush();
    }

    static Instrumentor& Get()
    {
        static Instrumentor* instance = new Instrumentor();
        return *instance;
    }
};

class InstrumentationTimer
{
public:
    InstrumentationTimer(const char* name)
        : m_Name(name), m_Stopped(false)
    {
        m_StartTimepoint = std::chrono::system_clock::now();
    }

    ~InstrumentationTimer()
    {
        if (!m_Stopped)
            Stop();
    }

    void Stop()
    {
        auto endTimepoint = std::chrono::system_clock::now();

        long long start = std::chrono::time_point_cast<std::chrono::microseconds>(m_StartTimepoint).time_since_epoch().count();
        long long end = std::chrono::time_point_cast<std::chrono::microseconds>(endTimepoint).time_since_epoch().count();

        uint32_t threadID = std::hash<std::thread::id>{}(std::this_thread::get_id());
        Instrumentor::Get().WriteProfile({ m_Name, start, end, threadID });

        m_Stopped = true;
    }
private:
    const char* m_Name;
    std::chrono::system_clock::time_point m_StartTimepoint;
    bool m_Stopped;
};


#endif
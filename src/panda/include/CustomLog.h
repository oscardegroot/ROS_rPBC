

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

#define LOG_LEVEL 2

#define DEBUG 3
#define INIT 2
#define WARNING 1
#define ERROR 0

#define INIT_COLOR "\033[0;32m"
#define ERROR_COLOR "\033[0;31m"
#define WARNING_COLOR "\033[0;33m"
#define END_ESC "\033[0m"

template <class T> 
void logMsg(std::string from_class, T msg, int level=3){

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
void logTmp(T msg){

	std::cout << "Tmp Logging: \n" << msg << std::endl;
}

void logMsg(std::string from_class, std::string msg, int level=3);

#endif
#include "CustomLog.h"

/* Levels
3 - Debug
2 - Warnings
1 - Errors
0 - Critical
*/



void logMsg(std::string from_class, std::string msg, int level){
	logMsg<std::string>(from_class, msg, level);
}

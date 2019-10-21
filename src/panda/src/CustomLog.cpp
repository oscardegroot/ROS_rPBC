#include "CustomLog.h"

/* Levels
3 - Debug
2 - Warnings
1 - Errors
0 - Critical
*/



void logMsg(const std::string& from_class, const std::string& msg, int level){
	logMsg<std::string>(from_class, msg, level);
}

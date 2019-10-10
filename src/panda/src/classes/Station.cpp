//#include "Station.h"
//
//
//Station::Station()
//{
//    
//    double register_time;
//    helpers::safelyRetrieve(nh, "/register_time", register_time);
//    
//    // Initiate a timer for register duration
//    timer = std::make_unique<helpers::SimpleTimer>(register_time);
//
//}
//
//void Station::run(){
//    
//    switch(state){
//        case REGISTERING:
////            if(finishedRegistering()){
////                state = PREPARING;
////                
////            }
//            break;
//            
//        case STARTING:
//            starting();
//            break;
//            
//        case WAIT_FOR_RESPONSE:
//            
//            break;
//            
//        case ENABLE_STATION:
//            enableStation();
//            break;
//            
//        case RUNNING:
//            update();
//            break;            
//    }
//    
//    ros::spinOnce();
//}
//
//bool Station::finishedRegistering(){
//    return timer->finished();
//}
//
//

#pragma once

#include <px4_log.h>
#include <math.h>
#include <float.h>

//#ifdef CONFIG_DEBUG
//#define	__DEBUG__							(1)
//#else/*CONFIG_DEBUG*/
//#undef	__DEBUG__	
//#endif/*CONFIG_DEBUG*/

#if __MAVLINK_LOG_FILE__
#define xia_debug(_fd, _text, ...)			mavlink_vasprintf(_fd, MAVLINK_IOC_SEND_TEXT_INFO, _text, ##__VA_ARGS__)	\
											fprintf(stderr, _text, ##__VA_ARGS__);										\
											fprintf(stderr, "\n");
#else	
#define xia_debug(_fd, _text, ...)			fprintf(stderr, _text, ##__VA_ARGS__);	\
											fprintf(stderr, "\n");

#endif/*__MAVLINK_LOG_FILE__*/

#define __DEBUG__                               (0)
#define __DAVID_ARMED_FIX__                     (1)//add armed status
#define __FMU_CONFIG__  						(1)//to use the fmu to do the function of gpio
#define __ATT_PROTECT_FIX__  					(1)//to fix the att bug
#define __BATT_SERIAL__ 						(1)//to use new serial to read battery information
#define __DAVID_STATE_FIX__                     (1)


#define __DISTANCE_KS103__ 				    	(0)//to change mb12xx to ks103
#define __DISTANCE_FILTER__ 					(0)//to filter the up-ultrasonic data


#define __DAVID_YAW_FIX__  						(0)//to fix the yaw bug
#define __ARMED_FIX_1__  						(0)//to lock the stick value when disarming plane
#define __DAVID_NAV_LOG__  						(0)//to add nav log

#define __DAVID_DISTANCE__                      (0)//chaoshengbo control height
#define __DAVID_DISTANCE_FIX__                  (0)//chaoshengbo control height
#define __DAVID_CHAO_WARING__                   (0)//chaoshengbo front back low warning

#if __DEBUG__
#define PX4_ZK(FMT, ...) 						__px4_log_modulename(_PX4_LOG_LEVEL_ALWAYS, FMT, ##__VA_ARGS__)
#else
#define PX4_ZK(FMT, ...) 
#endif



//#define COMPARE_FLOAT(a,b)	((fabsf((float)(a) - (float)(b)) < FLT_EPSILON) ? 1 : 2)

//static __inline int __COMPARE_FLOAT(double _AA, double _BB)
//{
//	if (fabs(_AA - _BB)< DBL_EPSILON)
//	{
//	 	return 3;

//	} /* End if () */

//	if (_AA < _BB)
//	{
//		return -4;

//	} /* End if () */

// 	return 5;
//}


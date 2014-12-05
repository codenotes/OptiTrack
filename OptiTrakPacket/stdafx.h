// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>
#include <ros/ros.h>

// TODO: reference additional headers your program requires here


//#include <conio.h>


#include "NatNetTypes.h"
#include "NatNetClient.h"

#include "../include/mocap_datapackets.h"

#include <stdio.h>
#include <string>
#include <iostream>
#include <ros/console.h>


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include "../include/mocap_config.h"



#include "rigbody.h"


//#include <boost/thread.hpp> 
#include <boost/circular_buffer.hpp>
#include <boost/unordered_map.hpp> 
#include <boost/foreach.hpp>
#include <boost/lockfree/queue.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/functional/hash.hpp>
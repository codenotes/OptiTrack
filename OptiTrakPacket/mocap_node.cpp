/// \author <a href="mailto:graeve@ais.uni-bonn.de">Kathrin Gräve</a>
///
/// ROS node that translates motion capture data from an OptiTrack rig to tf transforms.
/// The node receives the binary packages that are streamed by the Arena software,
/// decodes them and broadcasts the poses of rigid bodies as tf transforms.
///
/// Currently, this node supports the NatNet streaming protocol v1.4.



//#include "../include/mocap_datapackets.h"
//#include "../include/mocap_config.h"
//#include "../include/skeletons.h"
//
//// ROS includes
//#include <ros/ros.h>
//#include <tf/transform_datatypes.h>
//#include <tf/transform_broadcaster.h>
//#include <geometry_msgs/Pose2D.h>
//
//// System includes
//#include <string>



// Local includes
//#include "../include/socket.h"
//#include <unistd.h>
#include "stdafx.h"
#include "../include/skeletons.h"

////////////////////////////////////////////////////////////////////////
// Constants

// ip on multicast group - cannot be changed in Arena
const std::string MULTICAST_IP = "224.0.0.1";

const std::string MOCAP_MODEL_KEY = "mocap_model";
const std::string RIGID_BODIES_KEY = "rigid_bodies";
const char ** DEFAULT_MOCAP_MODEL = SKELETON_WITHOUT_TOES;

const int LOCAL_PORT = 1511;

typedef boost::shared_ptr < RigidBody> _spRigBody;
extern boost::circular_buffer < _spRigBody > qRigBody;
boost::mutex mocap_mutex;
////////////////////////////////////////////////////////////////////////

int natnet_go(int argc, _TCHAR* argv[]);

void processMocapData( const char** mocap_model, RigidBodyMap& published_rigid_bodies)
{
#if 1
  //UdpMulticastSocket multicast_client_socket( LOCAL_PORT, MULTICAST_IP );

  unsigned short payload;
 // int numberOfPackets = 0;
  while(ros::ok())
  {
   // bool packetread = false;
   // int numBytes = 0;

    do
    {
      // Receive data from mocap device
     // numBytes = multicast_client_socket.recv();

      // Parse mocap data
      if( true) //numBytes > 0 )
      {
		  const char* buffer=0;// = multicast_client_socket.getBuffer();
        //unsigned short header = *((unsigned short*)(&buffer[0]));

        // Look for the beginning of a NatNet package
        if (true) //header == 7)
        {
          payload = *((unsigned short*) &buffer[2]);
          MoCapDataFormat format(buffer, payload);
          format.parse();

		  //GET THE DATA AND TURN IT INTO format.model.rigidBodies[] array.
		  //this will all happen in parse. 


          //packetread = true;
          //numberOfPackets++;

          if(true)// format.model.numRigidBodies > 0 )
          {
           for( int i = 0; i < format.model.numRigidBodies; i++ )
            {
              int ID = format.model.rigidBodies[i].ID;
              RigidBodyMap::iterator item = published_rigid_bodies.find(ID);

              if (item != published_rigid_bodies.end())
              {
				  //so here the data from the format is passed into the PublishedRigidBody function, and inside the data is transfered
				  //therefore, if I can provide the information that would otherwise be in format and pass it in here, I would usurp this
                  item->second.publish(format.model.rigidBodies[i]);
              }
            }
          }
        }
        // else skip packet
      }
	} while (true) //numBytes > 0 );
		;
    // Don't try again immediately
    //if( !packetread )
    //{
    //  usleep( 10 );
    //}
  }
#endif

}

void processMocapData2(RigidBodyMap& published_rigid_bodies)
{

	ROS_INFO_NAMED("interop", "Publishing...");

	ros::Rate loop_rate(75); //need a number high enough that eats into the buffer a little.
//	ros::Rate loop_rate_idle(1);

	
	while (ros::ok())
	{


	//	boost::mutex::scoped_lock lock(mocap_mutex);
		_spRigBody sp;

		//ROS_INFO_NAMED("interop", "sdf");

		if (qRigBody.size() > 0)
		{

			boost::circular_buffer < _spRigBody >::iterator it;

			//for (it= qRigBody.begin(); it != qRigBody.end(); ++it)
		//	{
			//	ROS_INFO_NAMED("interop", "dump ID %d", it->get()->ID);
			//}

			sp = qRigBody[0];

			//ROS_INFO_NAMED("interop", "popped ID %d, size %d", sp->ID, qRigBody.size());
			int ID = sp->ID;

			RigidBodyMap::iterator item = published_rigid_bodies.find(ID);

			if (item != published_rigid_bodies.end())
			{

				ROS_DEBUG_NAMED("interop", "Found and publishing ID %d, qsize is %d", ID, qRigBody.size());

				//so here the data from the format is passed into the PublishedRigidBody function, and inside the data is transfered
				//therefore, if I can provide the information that would otherwise be in format and pass it in here, I would usurp this
				item->second.publish(*sp);
			}

			qRigBody.pop_front();



			//printf("***********x is %f\n", sp->pose.position.x);
		}
		else
			continue;


	

		ros::spinOnce();
		loop_rate.sleep();
	}


	


}


////////////////////////////////////////////////////////////////////////

int main( int argc, char* argv[] )
{ 
  
 //ROS_DLL_LOCATION = C:\Users\Gregory Brill\Source\Repos\ROSIndigo\ROSIndigoDLL\Debug\
	
	

  // Initialize ROS node
  ros::init(argc, argv, "mocap_node");
  ros::NodeHandle n;// ("~");

  // Get configuration from ROS parameter server  
   const char** mocap_model =  DEFAULT_MOCAP_MODEL ;

  
   if( n.hasParam( MOCAP_MODEL_KEY ) )
  {    std::string tmp;
    if( n.getParam( MOCAP_MODEL_KEY, tmp ) )
    {
      if( tmp == "SKELETON_WITH_TOES" )
        mocap_model = SKELETON_WITH_TOES;
      else if( tmp == "SKELETON_WITHOUT_TOES" )
        mocap_model = SKELETON_WITHOUT_TOES;
      else if( tmp == "OBJECT" )
        mocap_model = OBJECT;
    }
  }

  RigidBodyMap published_rigid_bodies;


  printf(getenv("ROS_MASTER_URI"));



  if (n.hasParam("rigid_bodies"));// RIGID_BODIES_KEY))
  {
      XmlRpc::XmlRpcValue body_list;
      bool b=n.getParam("rigid_bodies", body_list);

      if (body_list.getType() == XmlRpc::XmlRpcValue::TypeStruct && body_list.size() > 0)
      {
          XmlRpc::XmlRpcValue::iterator i;
          for (i = body_list.begin(); i != body_list.end(); ++i) {
              if (i->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                  PublishedRigidBody body(i->second);
                  string id = (string&) (i->first);
                  RigidBodyItem item(atoi(id.c_str()), body);

                  std::pair<RigidBodyMap::iterator, bool> result = published_rigid_bodies.insert(item);
                  if (!result.second)
                  {
                      ROS_ERROR("Could not insert configuration for rigid body ID %s", id.c_str());
                  }
              }
          }
      }
	  natnet_go(argc, argv);
	  processMocapData2(published_rigid_bodies);

  }

  // Process mocap data until SIGINT
  //processMocapData(mocap_model, published_rigid_bodies);

  return 0;
}





//original processMocapData below
/*

UdpMulticastSocket multicast_client_socket( LOCAL_PORT, MULTICAST_IP );

ushort payload;
int numberOfPackets = 0;
while(ros::ok())
{
bool packetread = false;
int numBytes = 0;

do
{
// Receive data from mocap device
numBytes = multicast_client_socket.recv();

// Parse mocap data
if( numBytes > 0 )
{
const char* buffer = multicast_client_socket.getBuffer();
unsigned short header = *((unsigned short*)(&buffer[0]));

// Look for the beginning of a NatNet package
if (header == 7)
{
payload = *((ushort*) &buffer[2]);
MoCapDataFormat format(buffer, payload);
format.parse();

//GET THE DATA AND TURN IT INTO format.model.rigidBodies[] array.
//this will all happen in parse.


packetread = true;
numberOfPackets++;

if( format.model.numRigidBodies > 0 )
{
for( int i = 0; i < format.model.numRigidBodies; i++ )
{
int ID = format.model.rigidBodies[i].ID;
RigidBodyMap::iterator item = published_rigid_bodies.find(ID);

if (item != published_rigid_bodies.end())
{
item->second.publish(format.model.rigidBodies[i]);
}
}
}
}
// else skip packet
}
} while( numBytes > 0 );

// Don't try again immediately
if( !packetread )
{
usleep( 10 );
}
}

*/
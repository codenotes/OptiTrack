#pragma once
class RigBody
{
public:
	int ID;                                 // RigidBody identifier
	float x, y, z;                          // Position
	float qx, qy, qz, qw;                   // Orientation
	int nMarkers;                           // Number of markers associated with this rigid body
	MarkerData* Markers;                    // Array of marker data ( [nMarkers][3] )
	int* MarkerIDs;                         // Array of marker IDs
	float* MarkerSizes;                     // Array of marker sizes
	float MeanError;                        // Mean measure-to-solve deviation
	short params;                           // Host defined tracking flags


public:
	RigBody();
	~RigBody();
};


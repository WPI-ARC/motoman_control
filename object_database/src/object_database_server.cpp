#include <iostream>
#include <string>
#include <sys/types.h>
#include <dirent.h>
#include <string>
#include <errno.h>
#include <algorithm>
#include <unordered_map>

#include "ros/ros.h"
#include "object_database/DatabaseRetrievalService.h"
#include "ros/package.h"

#include <vector>

using namespace std;

/*
 * \class Object
 *  Contains the properties of a given object
 *  TODO: Add more properties(such as features, 3D models) of the object as data members
 *  TODO: Move object definition to its own header file
 */
class Object
{
private:

	int ID;
	string Name;

public:

	// ctr
	Object(int id, string name):  ID(id), Name(name){}
	Object():		 ID(0), Name(""){}

	// getters and setters
	string getName() const { return Name; }
	void setName(string name) { Name = name; }

	int getID() const { return ID; }
	void setID(int id){ ID = id; }

	// dtr
	~Object(){}

};

/*
 * Hash map for retrieving the value as an instance of an object given its key name as string
 */
unordered_map<string, Object> DatabaseMap;

/*
 * \fn bool initDatabase()
 * \brief Initializes the database with its properties that reside in a folder/directory
 * \param strDirectory a string defining the path to the directory where each object's properties are stored
 * \return success as a boolean
 */
bool initDatabase(string strDirectory)
{
	 DIR *dp;
	 struct dirent *dirp;

	 int id = 1;

	 // Open the directory
	 if((dp = opendir(strDirectory.c_str())) != NULL)
	 {
		 // Read the sub-directories
		 while((dirp = readdir(dp)) != NULL)
		 {
			 string strTempFilename = string(dirp->d_name);
			 if(strcmp(strTempFilename.c_str(), ".") != 0 && strcmp(strTempFilename.c_str(), "..") != 0)
			 {
				 // Extract name of the sub-directories (objects) as lower case for standardization purposes
				 string objectName(dirp->d_name);
				 std::transform(objectName.begin(), objectName.end(), objectName.begin(), ::tolower);

				 //Initialize the database with the name and instance of the object
				 DatabaseMap[objectName] = Object(id++, objectName);
//				 cout<<objectName<<"\n";
			 }
		 }
		 return true;
	 }
	 else
	 {
		 ROS_INFO("cannot open directory. %s \n", strDirectory.c_str());
		 return false;
	 }
}

/*
 * \fn bool findObject(object_database::DatabaseRetrievalService::Request &req,
 *			     		object_database::DatabaseRetrievalService::Response &res)
 * \brief Provides the service to find the requested object within the database
 * \param req DatabaseRetrievalService defined in the service as a request
 * \param res DatabaseRetrievalService defined in the service as a response
 * \return success as a boolean
 * TODO: Add complex properties of the objects to be returned as a response
 */
bool findObject (object_database::DatabaseRetrievalService::Request &req,
		object_database::DatabaseRetrievalService::Response &res)
{
	unordered_map<string, Object>::const_iterator got = DatabaseMap.find (req.object_name);

	// If object not found
	if ( got == DatabaseMap.end() )
	{
		ROS_INFO("Not found");
		return false;
	}
	// Object found
	else
	{
		// return response as ID of the object
		res.id = (got->second).getID();
	}
	return true;
}

int main(int argc, char **argv)
{
	// Initialize the ROS node
	ros::init(argc, argv, "object_database_server");
	ros::NodeHandle n;

	std::string strDirectory = ros::package::getPath("object_database") + "/src/" + "objects";
	//	 cout<<strDirectory<<"\n";

	// Initialize the database
	initDatabase(strDirectory);

	// Advertize the service
	ros::ServiceServer service = n.advertiseService("object_database_server", findObject);
	ROS_INFO("Ready to find objects.");

	ros::spin();

	return 0;
}

// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"

using namespace std;
using namespace Eigen;

const string robot_file = "resources/panda_arm.urdf";

// redis keys:
const string JOINT_ANGLES_KEY = "sai2::FrankaPanda::Bonnie::sensors::q";

int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	Affine3d T_kuka_Bonnie = Affine3d::Identity();
	T_kuka_Bonnie.translation() = Vector3d(1.73603,  -0.421108, -0.0130478);
	T_kuka_Bonnie.linear() = AngleAxisd(-1.11014, Vector3d::UnitZ()).toRotationMatrix();
	auto robot = new Sai2Model::Sai2Model(robot_file, false, T_kuka_Bonnie);

	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0,0,0.211);

	Vector3d ee_pos_in_kuka_frame = Vector3d::Zero();

	bool runloop = true;

	while(runloop)
	{
		// wait for user input
		char s = getchar();
		if(s == 'q')
		{
			runloop = false;
			break;
		}

		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->updateModel();

		robot->positionInWorld(ee_pos_in_kuka_frame, link_name, pos_in_link);
		cout << "---------------------------------------------------------------" << endl;
		cout << ee_pos_in_kuka_frame.transpose() << endl;
		cout << "---------------------------------------------------------------" << endl;
		cout << endl << endl << endl;

	}

	return 0;
}

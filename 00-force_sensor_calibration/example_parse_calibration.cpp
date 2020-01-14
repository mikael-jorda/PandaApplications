#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <tinyxml2.h>

using namespace std;
using namespace Eigen;

VectorXd readBiasXML(const string path_to_bias_file);
void readToolMassCOMXML(double& tool_mass, Vector3d& tool_com, const string calibration_filename);

int main(int argc, char** argv) {

	if(argc < 3)
	{
		cout << "usage : ./example_parse_calibration [path_to_bias_file] [path_to_obj_calibration_file]" << endl;
		return 0;
	}
	const string bias_file = argv[1];
	const string calibration_file = argv[2];

	// force readings
	VectorXd bias_force = VectorXd::Zero(6);

	double tool_mass = 0;
	Vector3d tool_com = Vector3d::Zero();

	bias_force = readBiasXML(bias_file);
	readToolMassCOMXML(tool_mass, tool_com, calibration_file);

	cout << "\nbias :\n" << bias_force.transpose() << endl << endl;
	cout << "\ntool mass :\n" << tool_mass << "\ntool com :\n" << tool_com.transpose() << endl << endl;

	return 0;
}

VectorXd readBiasXML(const string path_to_bias_file)
{
	VectorXd sensor_bias = VectorXd::Zero(6);
	tinyxml2::XMLDocument doc;
	doc.LoadFile(path_to_bias_file.c_str());
	if (!doc.Error())
	{
		cout << "Loading bias file file ["+path_to_bias_file+"]." << endl;
		try 
		{

			std::stringstream bias( doc.FirstChildElement("force_bias")->
				Attribute("value"));
			bias >> sensor_bias(0);
			bias >> sensor_bias(1);
			bias >> sensor_bias(2);
			bias >> sensor_bias(3);
			bias >> sensor_bias(4);
			bias >> sensor_bias(5);
			std::stringstream ss; ss << sensor_bias.transpose();
			cout << "Sensor bias : "+ss.str() << endl;
		}
		catch( const std::exception& e ) // reference to the base of a polymorphic object
		{ 
			std::cout << e.what(); // information from length_error printed
			cout << "WARNING : Failed to parse bias file." << endl;
		}
	} 
	else 
	{
		cout << "WARNING : Could no load bias file ["+path_to_bias_file+"]" << endl;
		doc.PrintError();
	}
	return sensor_bias;
}

void readToolMassCOMXML(double& tool_mass, Vector3d& tool_com, const string calibration_filename)
{
	tinyxml2::XMLDocument doc;
	doc.LoadFile(calibration_filename.c_str());
	if (!doc.Error())
	{
		cout << "Loading tool file [" << calibration_filename << "]." << endl;
		try 
		{
			std::string mass = doc.FirstChildElement("tool")->
			FirstChildElement("inertial")->
			FirstChildElement("mass")->
			Attribute("value");
			tool_mass = std::stod(mass);
			cout << "Tool mass: " << mass << endl;

			std::stringstream com( doc.FirstChildElement("tool")->
				FirstChildElement("inertial")->
				FirstChildElement("origin")->
				Attribute("xyz"));
			com >> tool_com(0);
			com >> tool_com(1);
			com >> tool_com(2);
			std::stringstream ss; ss << tool_com.transpose();
			cout << "Tool CoM : " << ss.str() << endl;
		}
		catch( const std::exception& e ) // reference to the base of a polymorphic object
		{ 
			std::cout << e.what(); // information from length_error printed
			cout << "WARNING : Failed to parse tool file." << endl;
		}
	} 
	else 
	{
		cout << "WARNING : Could no load tool file [" << calibration_filename << "]" << endl;
		doc.PrintError();
	}
}
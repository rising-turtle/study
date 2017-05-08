#ifndef CONFIG_H
#define CONFIG_H

#include <fstream>
#include <string>
#include <map>

using namespace std;

class CConfig
{
public:
	CConfig(string file);
	~CConfig();
	string m_path_save; 
	bool m_bSickOdo; 
	vector<string> m_ips; 
	vector<string> m_ports;
};

#endif
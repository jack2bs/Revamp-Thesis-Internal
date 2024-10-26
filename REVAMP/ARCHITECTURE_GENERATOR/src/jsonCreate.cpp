/*
 * json.cpp
 *
 *      Author: thilini
 */

//#include "architecture.h"
//#include "DFG.h"
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <assert.h>
#include <fstream>
#include <nlohmann/json.hpp>
#include <iomanip>
#include <regex>
#include "jsonCreate.h"
#include <array>
#include <bits/stdc++.h>
#include "PE.h"
#include "architecture.h"
using namespace std;
using json = nlohmann::json;

namespace revamp
{
jsonCreate::jsonCreate()
{
	// TODO Auto-generated constructor stub
}

void jsonCreate::extractJson(std::string fileName)
{
	ifstream json_file(fileName.c_str());
	assert(json_file.is_open());
	json json1;
	json_file >> json1;

	this->dp = json1["ARCH"]["DP"];
	this->mdp = json1["ARCH"]["MDP"];
	this->fu = json1["ARCH"]["FU"];
	this->fu_mem = json1["ARCH"]["FU_MEM"];
	this->pe = json1["ARCH"]["PE"];
	this->pe_mem = json1["ARCH"]["PE_MEM"];
	this->cgra = json1["ARCH"]["CGRA"];
}

json jsonCreate::getStreamingDP(json &generated)
{
	generated["ARCH"]["DP"] =  {{"INPUTS",{"I1_LSB","I1_MSB","I2_LSB","I2_MSB","P"}},
				   {"INTERNALS",{"T_LSB_INT","T_MSB_INT"}},
				   {"OUTPUTS" ,{"T_LSB","T_MSB"}},
				   {"CONNECTIONS",{
						{"THIS.T_LSB_INT" ,{"THIS.T_LSB"}},
						{"THIS.T_MSB_INT" ,{"THIS.T_MSB"}}
			       	   	}
				   }
	};
	return generated;
}

json jsonCreate::getStreamingFU(json &top,json &generated,string type)
{
	//json generated;
	generated["ARCH"][type]["INPUTS"] = {"DP0_I1_LSB","DP0_I1_MSB","DP0_I2_LSB","DP0_I2_MSB","DP0P"};
	generated["ARCH"][type]["OUTPUTS"] = {"DP0_T_LSB","DP0_T_MSB"};
	generated["ARCH"][type]["OPS"] = top["OPS"];
	generated["ARCH"][type]["SUBMODS"] = top["SUBMODS"];
	generated["ARCH"][type]["CONNECTIONS"] ={"THIS.DP0_I1_LSB", {"DP0.I1_LSB"},
		    "THIS.DP0_I1_MSB" ,{"DP0.I1_MSB"},
	        "THIS.DP0_I2_LSB" , {"DP0.I2_LSB"},
		    "THIS.DP0_I2_MSB" , {"DP0.I2_MSB"},
	        "THIS.DP0_P" , {"DP0.P"},
	        "DP0.T_LSB" , {"THIS.DP0_T_LSB"},
		    "DP0.T_MSB" , {"THIS.DP0_T_MSB"}};

	return generated;
}

json jsonCreate::getHeteroFU(json &top,json &generated,string type,std::vector<std::string> supportedCompute,bool is_network,bool is_compute)
{
	if(is_compute)
	{
		//generated["ARCH"][type]["INPUTS"] = top["INPUTS"];
		//generated["ARCH"][type]["OUTPUTS"] = top["OUTPUTS"];
		//generated["ARCH"][type]["SUBMODS"] = top["SUBMODS"];
		//generated["ARCH"][type]["CONNECTIONS"] = top["CONNECTIONS"];
		for(std::string s:supportedCompute)
		{
			if(type=="FU_MEM")
			{
				generated["ARCH"][type]["OPS"][s]=2;
			}else
			{
				generated["ARCH"][type]["OPS"][s]=1;
			}
		}
	}else{
		generated["ARCH"][type]["OPS"] = top["OPS"];
	}
	if(is_network)
	{
		generated["ARCH"][type]["INPUTS"] = {"DP0_I1_LSB","DP0_I1_MSB","DP0_I2_LSB","DP0_I2_MSB","DP0_P"};
		generated["ARCH"][type]["OUTPUTS"] = {"DP0_T_LSB","DP0_T_MSB"};
		//generated["ARCH"][type]["OPS"] = top["OPS"];
		//generated["ARCH"][type]["SUBMODS"] = top["SUBMODS"];
		/*generated["ARCH"][type]["CONNECTIONS"] ={{"THIS.DP0_I1_LSB", {"DP0.I1_LSB"},
			    "THIS.DP0_I1_MSB" ,{"DP0.I1_MSB"},
		        "THIS.DP0_I2_LSB" , {"DP0.I2_LSB"},
			    "THIS.DP0_I2_MSB" , {"DP0.I2_MSB"},
		        "THIS.DP0_P" , {"DP0.P"},
		        "DP0.T_LSB" , {"THIS.DP0_T_LSB"},
			    "DP0.T_MSB" , {"THIS.DP0_T_MSB"}}};*/
		generated["ARCH"][type]["CONNECTIONS"]["THIS.DP0_I1_LSB"]={"DP0.I1_LSB"};
		generated["ARCH"][type]["CONNECTIONS"]["THIS.DP0_I1_MSB"]={"DP0.I1_MSB"};
		generated["ARCH"][type]["CONNECTIONS"]["THIS.DP0_I2_LSB"]={"DP0.I2_LSB"};
		generated["ARCH"][type]["CONNECTIONS"]["THIS.DP0_I2_MSB"]={"DP0.I2_MSB"};
		generated["ARCH"][type]["CONNECTIONS"]["THIS.DP0_P"]={"DP0.P"};
		generated["ARCH"][type]["CONNECTIONS"]["DP0.T_LSB"]={"THIS.DP0_T_LSB"};
		generated["ARCH"][type]["CONNECTIONS"]["DP0.T_MSB"]={"THIS.DP0_T_MSB"};
	}else
	{
		generated["ARCH"][type]["INPUTS"] = top["INPUTS"];
		generated["ARCH"][type]["OUTPUTS"] = top["OUTPUTS"];
		generated["ARCH"][type]["CONNECTIONS"] = top["CONNECTIONS"];
	}

	/*if(!is_compute && !is_network)
	{
		generated["ARCH"][type]["INPUTS"] = top["INPUTS"];
		generated["ARCH"][type]["OUTPUTS"] = top["OUTPUTS"];
		generated["ARCH"][type]["CONNECTIONS"] = top["CONNECTIONS"];
		generated["ARCH"][type]["OPS"] = top["OPS"];
	}*/
	generated["ARCH"][type]["SUBMODS"] = top["SUBMODS"];

	return generated;
}
//Cannot work for other architectures
json jsonCreate::getStreamingPE(json &top,json &generated,string type)
{


	generated["ARCH"][type]["INPUTS"] = top["INPUTS"];
	generated["ARCH"][type]["OUTPUTS"] = top["OUTPUTS"];
	generated["ARCH"][type]["INTERNALS"] = top["INTERNALS"];
	generated["ARCH"][type]["SUBMODS"] = top["SUBMODS"];
	//std::string s_r = top["REGS"].dump();
	std::size_t found,pos;
	/*for(auto &el:top["REGS"].items())
	{
		std::cout <<el.value().size()<<"\n";
		std::vector<std::string> rv = el.value();
		std::vector<std::string> arr;
		for(std::string el:rv)
		{

			if(((pos = it.find("TREG")) != std::string::npos))
			{
				arr.push_back(it+"_MSB");
				arr.push_back(it+"_LSB");
			}else
			{
				arr.push_back(it);
				arr.push_back(it);
			}
		}
		generated[type]["REGS"] = arr;
	}*/


	generated["ARCH"][type]["REGS"]={"NR","ER","WR","SR","TREG_LSB","TREG_MSB"};

	for(auto &el:top["CONNECTIONS"].items())
	{
		auto s = el.key();
		std::vector<std::string> sv = el.value();
		if((found = s.find("DP0_T")) != std::string::npos)
		{
			std::vector<std::string> arr_msb;
			std::vector<std::string> arr_lsb;
			for(auto it:sv)
			{
				if(((pos = it.find("TREG")) != std::string::npos))
				{
					arr_msb.push_back(it+"_MSB");
					arr_lsb.push_back(it+"_LSB");
				}else
				{
					arr_msb.push_back(it);
					arr_lsb.push_back(it);
				}
			}
			generated["ARCH"][type]["CONNECTIONS"][s+"_LSB"] = arr_lsb;
			generated["ARCH"][type]["CONNECTIONS"][s+"_MSB"] = arr_msb;
		}else if((found = s.find("TREG")) != std::string::npos)
		{
			std::vector<std::string> arr_msb;
			std::vector<std::string> arr_lsb;
			for(auto it:sv)
			{
				if(((pos = it.find("I1")) != std::string::npos) or ((pos = it.find("I2")) != std::string::npos))
				{
					arr_msb.push_back(it+"_MSB");
					arr_lsb.push_back(it+"_LSB");
				}else{
					arr_msb.push_back(it);
					arr_lsb.push_back(it);
				}
			}
			generated["ARCH"][type]["CONNECTIONS"][s+"_LSB"] = arr_lsb;
			generated["ARCH"][type]["CONNECTIONS"][s+"_MSB"] = arr_msb;
		}else
		{
			std::vector<std::string> arr;
			for(auto it:sv)
			{
				if(((pos = it.find("I1")) != std::string::npos) or ((pos = it.find("I2")) != std::string::npos))
				{
					arr.push_back(it+"_MSB");
					arr.push_back(it+"_LSB");
				}else{
					arr.push_back(it);
				}
			}
			generated["ARCH"][type]["CONNECTIONS"][s] = arr;

		}
	}

	return generated;
}

json jsonCreate::getHeteroPE(json &top,json &generated,string type,string fuType,bool is_network,bool is_compute,bool is_config)
{

	if(is_compute)
	{
		//generated["ARCH"][type]["INPUTS"] = top["INPUTS"];
		//generated["ARCH"][type]["OUTPUTS"] = top["OUTPUTS"];
		//generated["ARCH"][type]["INTERNALS"] = top["INTERNALS"];
		//generated["ARCH"][type]["REGS"] = top["REGS"];
		//generated["ARCH"][type]["CONNECTIONS"] = top["CONNECTIONS"];
		if(type=="PE_MEM")
		{
			generated["ARCH"][type]["SUBMODS"][fuType] = top["SUBMODS"]["FU_MEM"];
		}else{
			generated["ARCH"][type]["SUBMODS"][fuType] = top["SUBMODS"]["FU"];
		}
	}else
	{
		if(type=="PE_MEM")
		{
			generated["ARCH"][type]["SUBMODS"]["FU_MEM"] = top["SUBMODS"]["FU_MEM"];
		}else{
			generated["ARCH"][type]["SUBMODS"]["FU"] = top["SUBMODS"]["FU"];
		}
	}

	if(is_network)
	{
		//generated["ARCH"][type]["INPUTS"] = top["INPUTS"];
		//generated["ARCH"][type]["OUTPUTS"] = top["OUTPUTS"];
		//generated["ARCH"][type]["INTERNALS"] = top["INTERNALS"];
		//generated["ARCH"][type]["SUBMODS"] = top["SUBMODS"];
		std::size_t found,pos;

		generated["ARCH"][type]["REGS"]={"NR","ER","WR","SR","TREG_LSB","TREG_MSB"};

		for(auto &el:top["CONNECTIONS"].items())
		{
				auto s = el.key();
				std::vector<std::string> sv = el.value();
				if((found = s.find("DP0_T")) != std::string::npos)
				{
					std::vector<std::string> arr_msb;
					std::vector<std::string> arr_lsb;
					for(auto it:sv)
					{
						if(((pos = it.find("TREG")) != std::string::npos))
						{
							arr_msb.push_back(it+"_MSB");
							arr_lsb.push_back(it+"_LSB");
						}else
						{
							arr_msb.push_back(it);
							arr_lsb.push_back(it);
						}
					}
					generated["ARCH"][type]["CONNECTIONS"][s+"_LSB"] = arr_lsb;
					generated["ARCH"][type]["CONNECTIONS"][s+"_MSB"] = arr_msb;
				}else if((found = s.find("TREG")) != std::string::npos)
				{
					std::vector<std::string> arr_msb;
					std::vector<std::string> arr_lsb;
					for(auto it:sv)
					{
						if(((pos = it.find("I1")) != std::string::npos) or ((pos = it.find("I2")) != std::string::npos))
						{
							arr_msb.push_back(it+"_MSB");
							arr_lsb.push_back(it+"_LSB");
						}else{
							arr_msb.push_back(it);
							arr_lsb.push_back(it);
						}
					}
					generated["ARCH"][type]["CONNECTIONS"][s+"_LSB"] = arr_lsb;
					generated["ARCH"][type]["CONNECTIONS"][s+"_MSB"] = arr_msb;
				}else
				{
					std::vector<std::string> arr;
					for(auto it:sv)
					{
						if(((pos = it.find("I1")) != std::string::npos) or ((pos = it.find("I2")) != std::string::npos))
						{
							arr.push_back(it+"_MSB");
							arr.push_back(it+"_LSB");
						}else{
							arr.push_back(it);
						}
					}
					generated["ARCH"][type]["CONNECTIONS"][s] = arr;

				}
			}
	}else
	{
		generated["ARCH"][type]["REGS"] = top["REGS"];
		generated["ARCH"][type]["CONNECTIONS"] = top["CONNECTIONS"];
	}
	//TODO:: Make flexible per PE
	if(is_config)
	{
		//generated["ARCH"][type]["INPUTS"] = top["INPUTS"];
		//generated["ARCH"][type]["OUTPUTS"] = top["OUTPUTS"];
		//generated["ARCH"][type]["INTERNALS"] = top["INTERNALS"];
		//generated["ARCH"][type]["REGS"] = top["REGS"];
		//generated["ARCH"][type]["CONNECTIONS"] = top["CONNECTIONS"];
		//if(type=="PE_MEM")
		//{
		//	generated["ARCH"][type]["SUBMODS"]["FU_MEM"] = top["SUBMODS"]["FU_MEM"];
		//}else{
		//	generated["ARCH"][type]["SUBMODS"]["FU"] = top["SUBMODS"]["FU"];
		//}
		generated["ARCH"][type]["CONFIG_MEM"]["CONST"] = 4;
		generated["ARCH"][type]["CONFIG_MEM"]["OPCODE"] = 4;
		generated["ARCH"][type]["CONFIG_MEM"]["ROUTER"] = 8;

	}

	/*if(!is_config && !is_network && !is_compute)
	{
		generated["ARCH"][type]["INPUTS"] = top["INPUTS"];
		generated["ARCH"][type]["OUTPUTS"] = top["OUTPUTS"];
		generated["ARCH"][type]["INTERNALS"] = top["INTERNALS"];
		generated["ARCH"][type]["REGS"] = top["REGS"];
		generated["ARCH"][type]["CONNECTIONS"] = top["CONNECTIONS"];
		if(type=="PE_MEM")
		{
			generated["ARCH"][type]["SUBMODS"]["FU_MEM"] = top["SUBMODS"]["FU_MEM"];
		}else{
			generated["ARCH"][type]["SUBMODS"]["FU"] = top["SUBMODS"]["FU"];
		}
	}*/
	generated["ARCH"][type]["INPUTS"] = top["INPUTS"];
	generated["ARCH"][type]["OUTPUTS"] = top["OUTPUTS"];
	generated["ARCH"][type]["INTERNALS"] = top["INTERNALS"];

	return generated;
}

json jsonCreate::getHeteroCGRA(json &top,json &generated,std::vector<PE*> peArr,bool is_compute)
{
	//json generated;
	if(is_compute)
	{
	json PEConfig1;
	std::vector<json> PEjsonArr1;
	for(auto &el:top["SUBMODS"])
	{
		PEConfig1["PATTERN"]=el["PATTERN"];
		PEConfig1["DIMS"]=el["DIMS"];
		PEConfig1["CONNECTIONS"]=el["CONNECTIONS"];

	}

	json PEConfig;
	std::vector<json> PEjsonArr;
	int i=0;
	std::cout << "Json array1 size " << peArr.size() << "\n";
	for(PE* pe : peArr)
	{
		PEConfig["MOD"]=pe->peType;
		PEConfig["X"]=pe->x;
		PEConfig["Y"]=pe->y;

		PEjsonArr.push_back(PEConfig);

	}
	std::cout << "Json array size " << PEjsonArr.size() << "\n";
	PEConfig1["MODS"] = PEjsonArr;
	PEjsonArr1.push_back(PEConfig1);
	generated["ARCH"]["CGRA"]["SUBMODS"]=PEjsonArr1;
	}else{
		generated["ARCH"]["CGRA"]["SUBMODS"] =top["SUBMODS"];
	}

	return generated;
}

void jsonCreate::printJson(architecture *arch,std::string fileName,int index, bool is_network,bool is_compute,bool is_config,std::string opti)
{
	extractJson(fileName);
	json archi;
	string json_filename="generated_architectures/heterogeneous_CGRA_";

	if(is_compute){
		json_filename.append("compute_");
		json_filename.append(opti+"_");
	}

	if(is_network)
		json_filename.append("network_");
	if(is_config)
		json_filename.append("config_");

	json_filename.append(std::to_string(index) +".json");
	//if(is_network)
	//	json_filename = "heterogeneous_CGRA_" + std::to_string(index) +"_network.json";
	//else
	//	json_filename = "heterogeneous_CGRA_" + std::to_string(index) +".json";
	ofstream json_file(json_filename.c_str());

	extractJson(fileName);
	if(is_network)
		getStreamingDP(archi);
	else
		archi["ARCH"]["DP"] = this->dp;
	//archi["ARCH"]["MDP"] = this->mdp;

	std::vector<std::string> supportedCompute;
	if(is_compute)
	{
	std::cout << "COMP1\n";
		for(std::pair<std::string,std::vector<std::string>> pair:arch->computeFUType)
		{
			if(pair.first=="FU_MEM")
				getHeteroFU(this->fu_mem,archi,pair.first,pair.second,is_network, is_compute);
			else
				getHeteroFU(this->fu,archi,pair.first,pair.second,is_network, is_compute);
		}
	}else{
			getHeteroFU(this->fu_mem,archi,"FU_MEM",supportedCompute,is_network, is_compute);
			getHeteroFU(this->fu,archi,"FU",supportedCompute,is_network, is_compute);
	}

	if(is_compute)
	{
	std::cout << "COMP2\n";
		for(std::pair<std::string,std::string> s:arch->computeType)
		{
			if(s.first=="PE_MEM")
				getHeteroPE(this->pe_mem,archi,s.first,s.second, is_network, is_compute, is_config);
			else
				getHeteroPE(this->pe,archi,s.first,s.second, is_network, is_compute, is_config);
		}
	}else{
		getHeteroPE(this->pe_mem,archi,"PE_MEM","FU_MEM", is_network, is_compute, is_config);
		getHeteroPE(this->pe,archi,"PE","FU", is_network, is_compute, is_config);
	}
	std::cout << "Size " << arch->PEArr.size() << "\n" ;
	getHeteroCGRA(this->cgra,archi,arch->PEArr,is_compute);
	//std::cout << "Printing Started\n";
	//std::cout << "Size " << arch->PEArr.size() << "\n" ;
	json_file << setw(4) << archi << std::endl;
	json_file.close();
}

}

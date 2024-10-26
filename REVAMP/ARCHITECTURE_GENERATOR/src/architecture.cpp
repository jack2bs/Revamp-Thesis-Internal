/*
 * architecture.cpp
 *
 *      Author: thilini
 */
#include "architecture.h"
#include "DFG.h"
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
#include "PE.h"
#include <array>
#include <algorithm>
//#include "algorithm.h"
#include <bits/stdc++.h>

using namespace std;
using json = nlohmann::json;

namespace revamp
{

struct myclass {
  bool operator() (const pair<string, int> &a,const pair<string, int> &b){return (a.second > b.second);}
} myobject;


architecture::architecture()
{
	// TODO Auto-generated constructor stub
}

bool architecture::ParseJSONArch(std::string fileName)
{


	ifstream json_file(fileName.c_str());
	assert(json_file.is_open());
	json json;
	json_file >> json;


	PreprocessPattern(json["ARCH"]["CGRA"]);



	return true;
}

bool architecture::PreprocessPattern(json &top)
{
	json submods;
	json connections;

	for (auto &el : top["SUBMODS"])
	{

		if (el.find("PATTERN") != el.end())
		{
			assert(el["PATTERN"] == "GRID");
			int xdim = el["DIMS"]["X"];
			this->xmax = xdim;
			int ydim = el["DIMS"]["Y"];
			this->ymax = ydim;
			this->totalsize=xdim*ydim;
			this->memPEsize=0;

			int x,y;
			for (auto &ele : el["MODS"])
			{
				PE *pe =new PE();
				string mod_type = ele["MOD"];
				x=ele["X"];
				y=ele["Y"];

				pe->x = x;
				pe->y = y;
				if(mod_type=="PE_MEM")
				{
					this->memPEsize++;
					pe->is_mem = true;
				}else
				{
					pe->is_mem = false;
				}

				this->PEArr.push_back(pe);
			}

			//std::cout << "Total->" <<  this->totalsize << "  Memory->" << this->memPEsize << "\n";
		}
	}
	return true;
}

void architecture::calculateComputeRatio(opCount opcount,double effort[4])
{
	int total_hetero_count = opcount.add_count + opcount.logic_count + opcount.cmp_count + opcount.mul_count;

	int nonMemPE = this->totalsize - this->memPEsize;

	if(ceil((nonMemPE*effort[0]*opcount.add_count)/total_hetero_count) < nonMemPE)
	{
		this->heterocount.add_count = ceil((nonMemPE*effort[0]*opcount.add_count)/total_hetero_count);
	}else
	{
		this->heterocount.add_count = nonMemPE;
	}

	if(ceil((nonMemPE*effort[1]*opcount.cmp_count)/total_hetero_count) < nonMemPE)
	{
		this->heterocount.cmp_count = ceil((nonMemPE*effort[1]*opcount.cmp_count)/total_hetero_count);
	}else
	{
		this->heterocount.cmp_count = nonMemPE;
	}

	if(ceil((nonMemPE*effort[2]*opcount.logic_count)/total_hetero_count) < nonMemPE)
	{
		this->heterocount.logic_count = ceil((nonMemPE*effort[2]*opcount.logic_count)/total_hetero_count);
	}else
	{
		this->heterocount.logic_count = nonMemPE;
	}

	if(ceil((nonMemPE*effort[3]*opcount.mul_count)/total_hetero_count) < nonMemPE)
	{
		this->heterocount.mul_count = ceil((nonMemPE*effort[3]*opcount.mul_count)/total_hetero_count);
	}else
	{
		this->heterocount.mul_count = nonMemPE;
	}

	this->heterocount.mem_count = this->memPEsize;
	std::cout << "\nHeterogeneous Compute Distribution:\n";
	std::cout << "ADDER:" << this->heterocount.add_count << "\nMULTIPLIER:" << this->heterocount.mul_count <<"\nLOGIC:"<< this->heterocount.logic_count <<"\nCMP:" << this->heterocount.cmp_count <<"\nMemory:" << this->heterocount.mem_count<<"\n";
}

/*bool architecture::sortByVal(const pair<string, int> &a, 
               const pair<string, int> &b) 
{ 
    return (a.second < b.second); 
} */

std::vector<std::pair<string,unsigned int>> architecture::distributeOps()
{

	//MEMORY
	for(PE* pe: this->PEArr)
	{
			if(pe->hotspot_index==0)
			{
				pe->supportedCompute.push_back("MEMORY");
				pe->supportedCompute.push_back("ADDER");
			}

	}
	////////////////////////////////////////////////////////////////////////////
	std::vector<std::pair<string,unsigned int>> to_place;
        //for(int i=0 ; i<4;i++)
        //{
        to_place.push_back(make_pair("ADDER",this->heterocount.add_count));
	to_place.push_back(make_pair("COMPARATOR",this->heterocount.cmp_count));
	to_place.push_back(make_pair("LOGIC",this->heterocount.logic_count));
	to_place.push_back(make_pair("MULTIPLIER",this->heterocount.mul_count));

	std::sort(to_place.begin(), to_place.end(), myobject); 
	
	std::vector<PE*> possibleLoc;
        std::vector<std::pair<string,unsigned int>> to_place_rem;

        for(std::pair<string,unsigned int> pair:to_place)
        {
                possibleLoc = getPossiblePE(pair.first);
                std::cout << possibleLoc.size() <<"\n";
                std::cout <<"ZZ ::"<< pair.first << " "<<pair.second<<"\n";
                if(possibleLoc.size() < pair.second)
			std::cout <<"[ERROR]:Hotspot size is not sufficient, please increase the hotspot sizes."<<possibleLoc.size()<<pair.second<<"\n";
		assert(possibleLoc.size() >= pair.second);

                if(possibleLoc.size() == pair.second)
                {
                        for(PE* pe: this->PEArr)
                        {
                                if((std::find(possibleLoc.begin(),possibleLoc.end(),pe)!=possibleLoc.end()) && pe->supportedCompute.size() < pe->hotspot_index)
                                {
                                        //std::cout << "YY: " << pair.first << "\n";
                                        pe->supportedCompute.push_back(pair.first);
                                        pair.second--;
                                }
                        }
                        assert(pair.second==0);
                }else{
                        for(PE* pe: this->PEArr)
                        {
                                if(pe->hotspot_index==4 && pair.second>0)
                                {
                                        //std::cout << "XX: " << pair.first << "\n";
                                        pe->supportedCompute.push_back(pair.first);
                                        pair.second--;
                                }
                        }
                        if(pair.second>0)
                        {
                                to_place_rem.push_back(make_pair(pair.first,pair.second));
                                std::cout << pair.first << " " << pair.second << "\n";
                        }
                }
        }
	std::sort(to_place_rem.begin(), to_place_rem.end(), myobject);
        //}


	/*int arr [] ={this->heterocount.add_count,this->heterocount.cmp_count,this->heterocount.logic_count,this->heterocount.mul_count};
	sort(arr,arr+4,std::greater<int>());
	std::map<int,string> compute;
	compute[this->heterocount.add_count] = "ADDER";
	compute[this->heterocount.cmp_count] = "COMPARATOR";
	compute[this->heterocount.logic_count] = "LOGIC";
	compute[this->heterocount.mul_count] = "MULTIPLIER";

	std::vector<std::pair<string,unsigned int>> to_place;
	for(int i=0 ; i<4;i++)
	{
		to_place.push_back(make_pair(compute[arr[i]],arr[i]));
	}


	std::vector<PE*> possibleLoc;
	std::vector<std::pair<string,unsigned int>> to_place_rem;

	for(std::pair<string,unsigned int> pair:to_place)
	{
		possibleLoc = getPossiblePE(pair.first);
		//std::cout << possibleLoc.size() <<"\n";
	       	std::cout <<"ZZ ::"<< pair.first << " "<<pair.second<<"\n";
		if(possibleLoc.size() < pair.second)
			std::cout <<"[ERROR]:Hotspot size is not sufficient, please increase the hotspot sizes.\n";
		assert(possibleLoc.size() >= pair.second);

		if(possibleLoc.size() == pair.second)
		{
			for(PE* pe: this->PEArr)
			{
				if((std::find(possibleLoc.begin(),possibleLoc.end(),pe)!=possibleLoc.end()) && pe->supportedCompute.size() < pe->hotspot_index)
				{
					std::cout << "YY: " << pair.first << "\n";
					pe->supportedCompute.push_back(pair.first);
					pair.second--;
				}
			}
			assert(pair.second==0);
		}else{
			for(PE* pe: this->PEArr)
			{
				if(pe->hotspot_index==4 && pair.second>0)
				{
					std::cout << "XX: " << pair.first << "\n";
					pe->supportedCompute.push_back(pair.first);
					pair.second--;
				}
			}
			if(pair.second>0)
			{
				to_place_rem.push_back(make_pair(pair.first,pair.second));
				std::cout << pair.first << " " << pair.second << "\n";
			}
		}
	}
	int arrx [to_place_rem.size()];
	std::map<string,int> computex;
	for(int i=0;i<to_place_rem.size();i++)
	{
		arrx[i] = getPossiblePE(to_place_rem.at(i).first).size()-to_place_rem.at(i).second;
		computex[to_place_rem.at(i).first] = getPossiblePE(to_place_rem.at(i).first).size()-to_place_rem.at(i).second;
	}

	sort(arrx,arrx+to_place_rem.size());
	std::vector<std::pair<string,unsigned int>> to_place_rem_sorted;
	for(int i=0;i<to_place_rem.size();i++)
	{
		//std::cout << arrx[i] << "\n";
		for(std::pair<string,unsigned int> pair:to_place_rem){
			//if(pair.first == computex[arrx[i]])
			if((computex[pair.first] == arrx[i]) && (std::find(to_place_rem_sorted.begin(),to_place_rem_sorted.end(),pair) == to_place_rem_sorted.end() ))
			{
				to_place_rem_sorted.push_back(pair);

			}
		}
	}
	//std::cout << "XX: to_place_rem" << to_place_rem_sorted.size() << "\n";
	std::cout << "Sorted Remaining\n";
	for(std::pair<string,unsigned int> pair : to_place_rem_sorted)
	{
		std::cout << pair.first << " " << pair.second << "\n";
	}*/
/*	std::map<string,std::vector<std::vector<PE*>>> possiblePlacements;

	for(std::pair<string,unsigned int> pair:to_place_rem_sorted)
	{
		possibleLoc = getPossiblePE(pair.first);
		printCombination(possibleLoc,possibleLoc.size(), pair.second ,pair.first);

		for(int j=0;j<NUM_RAND;j++)
		{
			std::vector<PE*> selected = this->combinations_t.at(j);
		}
	}*/

	return to_place_rem;


}

std::vector<PE*> architecture::getPossiblePE(std::string compute)
{

	std::vector<PE*> possibleLoc;
	for(PE* pe: this->PEArr)
	{
		//std::cout << "PE-> x=" << pe->x << " y=" << pe->y << " filled=" << pe->supportedCompute.size()<< "hotspot="<< pe->hotspot_index << "compute="<<compute <<"\n";
		if(compute == "ADDER" && pe->hotspot_index > 0 && pe->supportedCompute.size() < pe->hotspot_index)
		{
			if(std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),"ADDER") == pe->supportedCompute.end())
				possibleLoc.push_back(pe);
		}

		else if((compute == "LOGIC" or compute == "COMPARATOR" )&& pe->hotspot_index > 1 && pe->supportedCompute.size() < pe->hotspot_index)
		{
			if((std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),"COMPARATOR") == pe->supportedCompute.end())&& compute == "COMPARATOR")
				possibleLoc.push_back(pe);
			if((std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),"LOGIC") == pe->supportedCompute.end())&& compute == "LOGIC")
				possibleLoc.push_back(pe);
		}
		else if(compute == "MULTIPLIER" && pe->hotspot_index > 2 && pe->supportedCompute.size() < pe->hotspot_index)
		{
			if(std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),"MULTIPLIER") == pe->supportedCompute.end())
				possibleLoc.push_back(pe);
		}

	}
	return possibleLoc;
}

void architecture::updatePE()
{
	for(PE* pe:this->PEArr)
	{
		pe->supportedOps.insert(pe->supportedOps.end(),pe->common.begin(),pe->common.end());
		if(pe->supportedCompute.size()==4)
		{
			pe->peType="PE";
			pe->fuType="FU";
			pe->supportedOps.insert(pe->supportedOps.end(),pe->adder.begin(),pe->adder.end());
			pe->supportedOps.insert(pe->supportedOps.end(),pe->comparator.begin(),pe->comparator.end());
			pe->supportedOps.insert(pe->supportedOps.end(),pe->logic.begin(),pe->logic.end());
			pe->supportedOps.insert(pe->supportedOps.end(),pe->multiplier.begin(),pe->multiplier.end());
		}else if(pe->hotspot_index==0)
		{
			pe->peType="PE_MEM";
			pe->fuType="FU_MEM";
			pe->supportedOps.insert(pe->supportedOps.end(),pe->adder.begin(),pe->adder.end());
			pe->supportedOps.insert(pe->supportedOps.end(),pe->memory.begin(),pe->memory.end());
		}
		else
		{
			std::string str="PE";
			std::string str1="FU";
			for(std::string s:pe->supportedCompute)
			{
				str = str + "_" + s;
				str1 = str1 + "_" + s;
				if(s=="ADDER")
					pe->supportedOps.insert(pe->supportedOps.end(),pe->adder.begin(),pe->adder.end());
				else if(s=="COMPARATOR")
					pe->supportedOps.insert(pe->supportedOps.end(),pe->comparator.begin(),pe->comparator.end());
				else if(s=="LOGIC")
					pe->supportedOps.insert(pe->supportedOps.end(),pe->logic.begin(),pe->logic.end());
				else if(s=="MULTIPLIER")
					pe->supportedOps.insert(pe->supportedOps.end(),pe->multiplier.begin(),pe->multiplier.end());
			}
			if(pe->supportedCompute.size()==0)
			{
				str = str + "_" + "COMMON";
                                str1 = str1 + "_" + "COMMON";
			}
			pe->peType=str;
			pe->fuType=str1;
		}
		if(std::find(this->computeType.begin(),this->computeType.end(),make_pair(pe->peType,pe->fuType))==this->computeType.end())
			this->computeType.push_back(make_pair(pe->peType,pe->fuType));
		if(std::find(this->computeFUType.begin(),this->computeFUType.end(),make_pair(pe->fuType,pe->supportedOps))==this->computeFUType.end())
			this->computeFUType.push_back(make_pair(pe->fuType,pe->supportedOps));
	}
}

void architecture::combinationUtil(std::vector<PE*> arr, PE* data [],int start, int end,int index, int r)
{

    if (index == r)
    {
        std::vector<PE*> dest(data, data + r);
        //std::cout << "new Combi\n";
        //for(PE* pe:dest)
        //	std::cout << "X->" << pe->x << " Y->" << pe->y << "\n";
        this->combinations_t.push_back(dest);
        return;
    }


    for (int i = start; i <= end && end - i + 1 >= r - index; i++)
    {
    	data[index] = arr.at(i);
        combinationUtil(arr, data, i+1,end, index+1, r);
    }
}

void architecture::printCombination(std::vector<PE*> arr, int n, int r,std::string type)
{
    PE* data[r];
    this->combinations_t.clear();
    combinationUtil(arr, data, 0, n-1, 0, r);
}

void architecture::printRTLConfig(int index, bool is_network, bool is_compute, bool is_config,std::string opti)
{
	string rtl_filename="generated_architectures_RTL_config/heterogeneous_CGRA_";
	if(is_compute)
	{
		rtl_filename.append("compute_");
		rtl_filename.append(opti+"_");
	}
	if(is_network)
		rtl_filename.append("network_");
	if(is_config)
		rtl_filename.append("config_");

	rtl_filename.append(std::to_string(index) +".vh");

	ofstream rtl_file(rtl_filename.c_str());

	string enable="parameter int ENABLE[";
	string router_config="parameter int ROUTER_CONFIG[";
	string opcode_config="parameter int OPCODE_CONFIG[";
	string const_config="parameter int CONST_CONFIG[";
	string local_config="parameter int LOCAL_CONFIG[";
	string alu_add_config="parameter int ALU_ADD[";
	string alu_cmp_config="parameter int ALU_CMP[";
	string alu_mul_config="parameter int ALU_MUL[";
	string alu_log_config="parameter int ALU_LOG[";
	string alu_mem_config="parameter int ALU_MEM[";
	enable.append(std::to_string(this->PEArr.size())+"] = {");
	router_config.append(std::to_string(this->PEArr.size())+"] = {");
	opcode_config.append(std::to_string(this->PEArr.size())+"] = {");
	const_config.append(std::to_string(this->PEArr.size())+"] = {");
	local_config.append(std::to_string(this->PEArr.size())+"] = {");
	alu_add_config.append(std::to_string(this->PEArr.size())+"] = {");
	alu_cmp_config.append(std::to_string(this->PEArr.size())+"] = {");
	alu_mul_config.append(std::to_string(this->PEArr.size())+"] = {");
	alu_log_config.append(std::to_string(this->PEArr.size())+"] = {");
	alu_mem_config.append(std::to_string(this->PEArr.size())+"] = {");

	for(PE* pe:this->PEArr)
	{
		enable.append(std::to_string(pe->enable_size)+",");
		router_config.append(std::to_string(pe->router_size)+",");
		opcode_config.append(std::to_string(pe->opcode_size)+",");
		const_config.append(std::to_string(pe->const_size)+",");
		local_config.append(std::to_string(pe->local_size)+",");

		if(std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),"ADDER") !=pe->supportedCompute.end())
		{
			alu_add_config.append("1,");
		}
		else if(!is_compute)
		{
			alu_add_config.append("1,");
		}else
		{
			alu_add_config.append("0,");
		}

		if(std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),"COMPARATOR") !=pe->supportedCompute.end())
		{
			alu_cmp_config.append("1,");
		}else if(!is_compute)
		{
			alu_cmp_config.append("1,");
		}else
		{
			alu_cmp_config.append("0,");
		}

		if(std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),"LOGIC") !=pe->supportedCompute.end())
		{
			alu_log_config.append("1,");
		}else if(!is_compute)
		{
			alu_log_config.append("1,");
		}else
		{
			alu_log_config.append("0,");
		}

		if(std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),"MULTIPLIER") !=pe->supportedCompute.end())
		{
			alu_mul_config.append("1,");
		}else if(!is_compute)
		{
			alu_mul_config.append("1,");
		}else
		{
			alu_mul_config.append("0,");
		}

		if(std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),"MEMORY") !=pe->supportedCompute.end())
		{
			alu_mem_config.append("1,");
		}else if(!is_compute && pe->is_mem)
		{
			alu_mem_config.append("1,");
		}else
		{
			alu_mem_config.append("0,");
		}
	}

	enable = enable.substr(0, enable.size()-1);
	enable.append("};");

	router_config = router_config.substr(0, router_config.size()-1);
	router_config.append("};");

	opcode_config = opcode_config.substr(0, opcode_config.size()-1);
	opcode_config.append("};");

	const_config = const_config.substr(0, const_config.size()-1);
	const_config.append("};");

	local_config = local_config.substr(0, local_config.size()-1);
	local_config.append("};");

	alu_add_config = alu_add_config.substr(0, alu_add_config.size()-1);
	alu_add_config.append("};");

	alu_cmp_config = alu_cmp_config.substr(0, alu_cmp_config.size()-1);
	alu_cmp_config.append("};");

	alu_log_config = alu_log_config.substr(0, alu_log_config.size()-1);
	alu_log_config.append("};");

	alu_mul_config = alu_mul_config.substr(0, alu_mul_config.size()-1);
	alu_mul_config.append("};");

	alu_mem_config = alu_mem_config.substr(0, alu_mem_config.size()-1);
	alu_mem_config.append("};");

	rtl_file << enable << "\n";
	rtl_file << router_config << "\n";
	rtl_file << opcode_config << "\n";
	rtl_file << const_config << "\n";
	rtl_file << local_config << "\n";
	rtl_file << alu_add_config << "\n";
	rtl_file << alu_cmp_config << "\n";
	rtl_file << alu_log_config << "\n";
	rtl_file << alu_mul_config << "\n";
	rtl_file << alu_mem_config << "\n";
	rtl_file.close();

}
void architecture::clear()
{
	for(PE* pe:this->PEArr)
	{
		pe->supportedCompute.clear();
		pe->supportedOps.clear();
	}
	this->combinations_t.clear();
	this->computeFUType.clear();
	this->computeType.clear();
}


}

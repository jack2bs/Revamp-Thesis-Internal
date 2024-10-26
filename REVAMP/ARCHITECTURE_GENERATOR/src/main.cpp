/*
 * main.cpp
 *
 *      Author: thilini
 */

#include <iostream>
#include <assert.h>
#include <string.h>
#include "DFG.h"
#include "architecture.h"
#include "main.h"
#include <math.h>

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include<dirent.h>
#include <string>
#include "jsonCreate.h"
#include <regex>
#include <iomanip>

using namespace std;
using namespace revamp;

//double Cfactor_typical[4] = {2.5,3.01,1.4,3.25}; //add,cmp,log,mul
double Cfactor_typical[4] = {2.5,2.5,2.5,2.5};
double Cfactor_high[4] = {2.5,2.5,2,2.5};
double Cfactor_low[4] = {1.5,1.5,1.5,1.5};

struct arguments
{
	string dfg_filename;
	string archi_filename;
	string config_filename;
};

/*bool sortByVal(const pair<string, int> &a,
               const pair<string, int> &b)
{
    return (a.second < b.second);
}*/

arguments parse_arguments(int argn, char *argc[])
{
	arguments ret;

	int c;

	opterr = 0;

	while (((c = getopt(argn, argc, "d:a:c:")) != -1))
		switch (c)
		{
		case 'd':
			ret.dfg_filename = string(optarg);
			break;
		case 'a':
			ret.archi_filename = string(optarg);
			break;
		case 'c':
			ret.config_filename = string(optarg);
			break;
		default:
			abort();
		}
	return ret;
}

std::string parseConfig(std::string fileName,architecture* archi, std::vector<PE*> PEArr)
{
	cout << "\nParsing config:\n";

	ifstream json_file(fileName.c_str());
	assert(json_file.is_open());
	json json;
	json_file >> json;

	for(auto &el : json["CONFIG"]["HOTSPOT"]["SUBMODS"])
	{
		if (el.find("PATTERN") != el.end())
		{
			assert(el["PATTERN"] == "GRID");
			int xdim = el["DIMS"]["X"];
			int ydim = el["DIMS"]["Y"];
			assert(archi->totalsize==xdim*ydim);

			for (auto &ele : el["MODS"])
			{

				int hot_index = ele["IDX"];

				for(PE* pe : PEArr)
				{
					if( ele["X"] == pe->x && ele["Y"] == pe->y)
					{
						pe->hotspot_index = hot_index;
					}
				}
			}
		}
	}

	//for(PE* pe : PEArr)
	//{

		//std::cout<< "X= " << pe->x << " Y=" << pe->y << " hotspot index=" << pe->hotspot_index << "\n";

	//}
	std::string effort = json["CONFIG"]["EFFORT"];

	cout << "Parsing config Success!!!\n";
	return effort;
}
void mergeCommon(architecture* src ,architecture* dest)
{
	for(int i=0;i< src->PEArr.size();i++)
	{
		dest->PEArr.at(i)->supportedCompute = src->PEArr.at(i)->supportedCompute;
		dest->PEArr.at(i)->hotspot_index = src->PEArr.at(i)->hotspot_index;
	}
}

void generateArchi(std::vector<std::pair<string,unsigned int>> to_place,architecture *ARCHI,std::string fileName, std::string opti)
{
	int other=3;
	int no_of_archi = std::pow(NUM_RAND,to_place.size())+other; //additional 3 for network and config
	std::cout << no_of_archi << "\n";
	jsonCreate jsonC;
	architecture instances[no_of_archi];
	std::vector<PE*> possibleLoc;
	std::cout << "Placing Started\n";
	std::cout <<to_place.size() <<"\n";
	if(to_place.size() >0)
	{
		for(int j=0; j<NUM_RAND;j++)
		{
			if(to_place.size() >1)
			{

				for(int k=0; k<NUM_RAND;k++)
				{
					if(to_place.size() >2)
					{
						for(int l=0; l<NUM_RAND;l++)
						{
							if(to_place.size() >3)
							{
								for(int m=0; m<NUM_RAND;m++)
								{
									int index = j*NUM_RAND*NUM_RAND*NUM_RAND +k*NUM_RAND*NUM_RAND+l*NUM_RAND+m;
									instances[index].ParseJSONArch(fileName);
									mergeCommon(ARCHI,&instances[index]);
									possibleLoc = instances[index].getPossiblePE(to_place.at(0).first);
									instances[index].printCombination(possibleLoc,possibleLoc.size(), to_place.at(0).second ,to_place.at(0).first);
									for(PE* pe:instances[index].combinations_t.at(m))
									{
										if(std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),to_place.at(0).first) == pe->supportedCompute.end())
											pe->supportedCompute.push_back(to_place.at(0).first);
									}
									possibleLoc = instances[index].getPossiblePE(to_place.at(1).first);
									instances[index].printCombination(possibleLoc,possibleLoc.size(), to_place.at(1).second ,to_place.at(1).first);
									for(PE* pe:instances[index].combinations_t.at(m))
									{
										if(std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),to_place.at(1).first) == pe->supportedCompute.end())
											pe->supportedCompute.push_back(to_place.at(1).first);
									}
									possibleLoc = instances[index].getPossiblePE(to_place.at(2).first);
									instances[index].printCombination(possibleLoc,possibleLoc.size(), to_place.at(2).second ,to_place.at(2).first);
									for(PE* pe:instances[index].combinations_t.at(m))
									{
										if(std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),to_place.at(2).first) == pe->supportedCompute.end())
											pe->supportedCompute.push_back(to_place.at(2).first);
									}
									possibleLoc = instances[index].getPossiblePE(to_place.at(3).first);
									instances[index].printCombination(possibleLoc,possibleLoc.size(), to_place.at(3).second ,to_place.at(3).first);
									for(PE* pe:instances[index].combinations_t.at(m))
									{
										if(std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),to_place.at(3).first) == pe->supportedCompute.end())
											pe->supportedCompute.push_back(to_place.at(3).first);
									}

								}
							}else
							{
								//std::cout << "Placing Started3\n";
								int index = j*NUM_RAND*NUM_RAND +k*NUM_RAND+l;
								instances[index].ParseJSONArch(fileName);
								mergeCommon(ARCHI,&instances[index]);
								possibleLoc = instances[index].getPossiblePE(to_place.at(0).first);
								instances[index].printCombination(possibleLoc,possibleLoc.size(), to_place.at(0).second ,to_place.at(0).first);
								//std::cout << to_place.at(0).first << " possible->" << possibleLoc.size() << "   " <<"combi->" << instances[index].combinations_t.size() << "\n";
								//std::cout << "combinations::" << instances[index].combinations_t.size() << "\n";
								for(PE* pe:instances[index].combinations_t.at(l))
								{
									if(std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),to_place.at(0).first) == pe->supportedCompute.end())
										pe->supportedCompute.push_back(to_place.at(0).first);
								}

								possibleLoc = instances[index].getPossiblePE(to_place.at(1).first);
								instances[index].printCombination(possibleLoc,possibleLoc.size(), to_place.at(1).second ,to_place.at(1).first);
								//std::cout << to_place.at(1).first << " possible->" << possibleLoc.size() << "   " <<"combi->" << instances[index].combinations_t.size() << "\n";
								//std::cout << "combinations::" << instances[index].combinations_t.size() << "\n";
								for(PE* pe:instances[index].combinations_t.at(l))
								{
								//	std::cout << "X->" << pe->x << " Y->" << pe->y << "\n";
									if(std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),to_place.at(1).first) == pe->supportedCompute.end())
									{
								//		std::cout << "X->" << pe->x << " Y->" << pe->y << "\n";
										pe->supportedCompute.push_back(to_place.at(1).first);
									}
								}
								possibleLoc = instances[index].getPossiblePE(to_place.at(2).first);
								instances[index].printCombination(possibleLoc,possibleLoc.size(), to_place.at(2).second ,to_place.at(2).first);
								//std::cout << to_place.at(2).first << " possible->" << possibleLoc.size() << "   " <<"combi->" << instances[index].combinations_t.size() << "\n";
								for(PE* pe:instances[index].combinations_t.at(l))
								{
									if(std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),to_place.at(2).first) == pe->supportedCompute.end())
									{
									//	std::cout << "X->" << pe->x << " Y->" << pe->y << "\n";
										pe->supportedCompute.push_back(to_place.at(2).first);
									}
								}

							}
						}

					}else
					{
						//std::cout << "Placing Started2\n";
						int index = j*NUM_RAND +k;
						instances[index].ParseJSONArch(fileName);
						mergeCommon(ARCHI,&instances[index]);
						possibleLoc = instances[index].getPossiblePE(to_place.at(0).first);
						instances[index].printCombination(possibleLoc,possibleLoc.size(), to_place.at(0).second ,to_place.at(0).first);
						for(PE* pe:instances[index].combinations_t.at(k))
						{
							if(std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),to_place.at(0).first) == pe->supportedCompute.end())
								pe->supportedCompute.push_back(to_place.at(0).first);
						}
						possibleLoc = instances[index].getPossiblePE(to_place.at(1).first);
						instances[index].printCombination(possibleLoc,possibleLoc.size(), to_place.at(1).second ,to_place.at(1).first);
						for(PE* pe:instances[index].combinations_t.at(k))
						{
							if(std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),to_place.at(1).first) == pe->supportedCompute.end())
								pe->supportedCompute.push_back(to_place.at(1).first);
						}
					}
				}
			}else
			{
				//std::cout << "Placing Started1\n";
				int index = j;
				instances[index].ParseJSONArch(fileName);
				mergeCommon(ARCHI,&instances[index]);
				possibleLoc = instances[index].getPossiblePE(to_place.at(0).first);
				instances[index].printCombination(possibleLoc,possibleLoc.size(), to_place.at(0).second ,to_place.at(0).first);
				for(PE* pe:instances[index].combinations_t.at(j))
				{
					if(std::find(pe->supportedCompute.begin(),pe->supportedCompute.end(),to_place.at(0).first) == pe->supportedCompute.end())
						pe->supportedCompute.push_back(to_place.at(0).first);
				}
			}

		}
	}
	// else
	// {
	// 	int index = 0;
	// 	instances[index].ParseJSONArch(fileName);
	// 	mergeCommon(ARCHI,&instances[index]);
	// 	possibleLoc = instances[index].getPossiblePE(to_place.at(0).first);
	// 	instances[index].printCombination(possibleLoc,possibleLoc.size(), to_place.at(0).second ,to_place.at(0).first);
	// }

	for(int j=0;j<no_of_archi;j++)
	{
		instances[no_of_archi-1-j].ParseJSONArch(fileName);
	}
	//std::cout << "Printing Started\n";
	for(int i=0;i<no_of_archi-other;i++)
	{
		instances[i].updatePE();
		jsonC.printJson(&instances[i],fileName, i, false,true,false,opti);
		//instances[i].printRTLConfig(i, false, true, false,opti);
		jsonC.printJson(&instances[i],fileName, i, true,true,false,opti);
		//instances[i].printRTLConfig(i, true, true, false,opti);
		jsonC.printJson(&instances[i],fileName, i, false,true,true,opti);
		//instances[i].printRTLConfig(i, false, true, true,opti);
		jsonC.printJson(&instances[i],fileName, i, true,true,true,opti);
		//instances[i].printRTLConfig(i, true, true, true,opti);
	}
		jsonC.printJson(&instances[no_of_archi-3],fileName, no_of_archi-3, true,false,true,"");
		instances[no_of_archi-3].printRTLConfig(no_of_archi-3, true, false, true,"");
		jsonC.printJson(&instances[no_of_archi-1],fileName, no_of_archi-1, false,false,true,"");
		instances[no_of_archi-1].printRTLConfig(no_of_archi-1, false, false, true,"");
		jsonC.printJson(&instances[no_of_archi-2],fileName, no_of_archi-2, true,false,false,"");
		instances[no_of_archi-2].printRTLConfig(no_of_archi-2, true, false, false,"");
		instances[0].printRTLConfig(0, false, true, false,opti);
		instances[0].printRTLConfig(0, true, true, false,opti);
		instances[0].printRTLConfig(0, false, true, true,opti);
		instances[0].printRTLConfig(0, true, true, true,opti);
}




int main(int argn, char *argc[])
{
	arguments args = parse_arguments(argn,argc);

	std::string inputDFG_folder = args.dfg_filename;
	std::string inputArchi_file = args.archi_filename;
	std::string inputConfig_file = args.config_filename;

	DFG currDFG;
	architecture ARCHI;


	opCount opcount;
	currDFG.initialize();

	struct dirent *d;
	DIR *dr;
	dr = opendir(inputDFG_folder.c_str());

	if(dr!=NULL)
	{
	  std::cout << "Total Number of Nodes:\n";
	  for(d=readdir(dr); d!=NULL; d=readdir(dr))
	  {
		  	  	std::string file = d->d_name;
	            if (file.find(".xml") != std::string::npos)
	            {
	            	currDFG.parseXML(inputDFG_folder+"/"+file);
	            	//currDFG.printDFG();
	            }
	  }
	  closedir(dr);

	  opcount = currDFG.analyzeOps();
	}
	else
	{
	  cout<<"\nError Occurred!";
	}
	cout << "\nParsing JSON:\n";
	ARCHI.ParseJSONArch(inputArchi_file);
	cout << "Parsing JSON Success!!!\n";
	std::string effort=parseConfig(inputConfig_file,&ARCHI,ARCHI.PEArr);
	std::cout << "Generate Heterogeneous Architectures";
	//if(effort =="low" or effort =="medium" or effort =="high")
	//{
		ARCHI.calculateComputeRatio(opcount,Cfactor_typical);
		std::vector<std::pair<string,unsigned int>> to_place=ARCHI.distributeOps();
		if(to_place.size()==0)
		{
			jsonCreate jsonC;
			ARCHI.updatePE();
	                jsonC.printJson(&ARCHI,inputArchi_file, 0, false,true,false,"L");
                	jsonC.printJson(&ARCHI,inputArchi_file, 0, true,true,false,"L");
                	jsonC.printJson(&ARCHI,inputArchi_file, 0, false,true,true,"L");
                	jsonC.printJson(&ARCHI,inputArchi_file, 0, true,true,true,"L");

					ARCHI.printRTLConfig(0, false, true, false,"L");
					ARCHI.printRTLConfig(0, true, true, false,"L");
					ARCHI.printRTLConfig(0, false, true, true,"L");
					ARCHI.printRTLConfig(0, true, true, true,"L");


		}else{
			std::cout << "Placing remaining\n";
			generateArchi(to_place,&ARCHI,inputArchi_file,"L");
		}
	//}
	if(effort =="medium" or effort =="high")
	{
		ARCHI.clear();
		to_place.clear();
		std::cout << "To Place size:: " << to_place.size() << "\n";
		ARCHI.calculateComputeRatio(opcount,Cfactor_high);
		to_place=ARCHI.distributeOps();
		std::cout << "To Place size:: " << to_place.size() << "\n";
		//std::cout << "Placing remaining\n";
		generateArchi(to_place,&ARCHI,inputArchi_file,"M");
	}
	if(effort =="high")
	{
		ARCHI.clear();
		ARCHI.calculateComputeRatio(opcount,Cfactor_low);
		std::vector<std::pair<string,unsigned int>> to_place=ARCHI.distributeOps();
		//std::cout << "Placing remaining\n";
		generateArchi(to_place,&ARCHI,inputArchi_file,"H");
	}


	return 0;
}





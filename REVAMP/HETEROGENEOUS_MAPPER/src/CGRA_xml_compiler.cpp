//============================================================================
// Name        : CGRA_xml_compiler.cpp
// Author      : Manupa Karunaratne
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
// Edited By: Thilini
//============================================================================

#include <iostream>
#include <assert.h>
#include <string.h>

#include "DFG.h"
#include "CGRA.h"
#include "HeuristicMapper.h"
#include "PathFinderMapper.h"
#include <math.h>

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

using namespace std;
using namespace CGRAXMLCompile;

//#define DUAL_STREAMING

struct arguments
{
	string dfg_filename;
	int xdim = -1;
	int ydim = -1;
	string PEType;
	string json_file_name;
	int userII = 0;
	bool noMutexPaths=false;
	int backtracklimit = 0;
	bool use_json = false;
	int ndps = 1;
	int maxiter = 30;
	int max_hops = 4;
	int maxII=25;
};

arguments parse_arguments(int argn, char *argc[])
{
	arguments ret;

	int aflag = 0;
	int bflag = 0;
	char *cvalue = NULL;
	int index;
	int c;

	opterr = 0;

	while ((c = getopt(argn, argc, "d:x:y:t:j:i:eb:m:h:q:")) != -1)
		switch (c)
		{
		case 'd':
			ret.dfg_filename = string(optarg);
			break;
		case 'x':
			ret.xdim = atoi(optarg);
			break;
		case 'y':
			ret.ydim = atoi(optarg);
			break;
		case 't':
			ret.PEType = string(optarg);
			break;
		case 'n':
			ret.ndps = atoi(optarg);
			break;
		case 'j':
			ret.json_file_name = string(optarg);
			ret.use_json = true;
			break;
		case 'i':
			ret.userII = atoi(optarg);
			break;
		case 'e':
			ret.noMutexPaths = true;
			break;
		case 'b':
			ret.backtracklimit = atoi(optarg);
			break;
		case 'm':
			ret.maxiter = atoi(optarg);
			break;
		case 'h':
			ret.max_hops = atoi(optarg);
			break;
		case 'q':
			ret.maxII = atoi(optarg);
			break;
		case '?':
			if (optopt == 'c')
				fprintf(stderr, "Option -%c requires an argument.\n", optopt);
			else if (isprint(optopt))
				fprintf(stderr, "Unknown option `-%c'.\n", optopt);
			else
				fprintf(stderr,
						"Unknown option character `\\x%x'.\n",
						optopt);
		default:
			abort();
		}
		return ret;
}

int main(int argn, char *argc[])
{
	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!

	// if (argn < 7)
	// {
	// 	std::cout << "arguments : <DFG.xml> <peType::\nGENERIC_8REGF,\nHyCUBE_8REGF,\nHyCUBE_4REG,\nN2N_4REGF,\nN2N_8REGF,\nSTDNOC_8REGF,\nSTDNOC_4REGF,\nSTDNOC_4REG,\nSTDNOC_4REGF_1P\nMFU_HyCUBE_4REG\nMFU_HyCUBE_4REGF\nMFU_STDNOC_4REG\nMFU_STDNOC_4REGF> <XYDim> <numberofDPS> <backtracklimit> <initII> <-arch_json> <-noMTpath>\n";
	// }
	// assert(argn >= 7);

	arguments args = parse_arguments(argn,argc);
	std::string inputDFG_filename = args.dfg_filename;\
	int xdim = args.xdim;
	int ydim = args.ydim;
	string PEType = args.PEType;
	int numberOfDPs = args.ndps;
	string json_file_name = args.json_file_name;
	int initUserII = args.userII;
	int maxII = args.maxII;
	
	DFG currDFG;
	currDFG.parseXML(inputDFG_filename);
	//currDFG.printDFG();

	bool isGenericPE;

	// CGRA testCGRA(NULL, "testCGRA", 1, ydim, xdim, &currDFG, PEType, numberOfDPs);

	CGRA *testCGRA;
	if (!args.use_json)
	{
		testCGRA = new CGRA(NULL, "coreCGRA", 1, ydim, xdim, &currDFG, PEType, numberOfDPs);
	}
	else
	{
		testCGRA = new CGRA(json_file_name, 1,xdim,ydim);
	}

	TimeDistInfo tdi = testCGRA->analyzeTimeDist();

	//	HeuristicMapper hm(inputDFG_filename);
	PathFinderMapper hm(inputDFG_filename);
	hm.setMaxIter(args.maxiter);

	int II = hm.getMinimumII(testCGRA, &currDFG);
	std::cout << "Minimum II = " << II << "\n";

	II = std::max(initUserII, II);

	std::cout << "Using II = " << II << "\n";

	hm.enableMutexPaths = true;
	if (args.noMutexPaths)
	{
		hm.enableMutexPaths = false;
	}
	hm.enableBackTracking = true;
	hm.backTrackLimit = args.backtracklimit;

	cout << "json_file_name = " << json_file_name << "\n";
	// exit(EXIT_SUCCESS);

	bool mappingSuccess = false;
	while (!mappingSuccess)
	{
		DFG tempDFG;
		tempDFG.parseXML(inputDFG_filename);
		//tempDFG.printDFG();

		CGRA *tempCGRA;
		if (json_file_name.empty())
		{
			tempCGRA = new CGRA(NULL, "coreCGRA", II, ydim, xdim, &tempDFG, PEType, numberOfDPs, hm.getcongestedPortsPtr());
		}
		else
		{
			tempCGRA = new CGRA(json_file_name, II,xdim,ydim, hm.getcongestedPortsPtr());
		}
		tempCGRA->analyzeTimeDist(tdi);
		tempCGRA->max_hops = args.max_hops;

		hm.getcongestedPortsPtr()->clear();
		hm.getconflictedPortsPtr()->clear();
		mappingSuccess = hm.Map(tempCGRA, &tempDFG);
		//hm.congestionInfoFile.close();
		if (!mappingSuccess)
		{

			for (DFGNode &node : currDFG.nodeList)
			{
				assert(node.rootDP == NULL);
			}

			delete tempCGRA;
			II++;

			if (II == maxII+5)
			{
				//std::cout << "II max of 65 has been reached and exiting...\n";
				std::cout << "MAPPING_RESULT :: FAIL II=0" << "\n";
				return 0;
			}

			if (II > hm.upperboundII)
			{
				std::cout << "upperbound II reached : " << hm.upperboundII << "\n";
				std::cout << "Please use the mapping with II = " << hm.upperboundFoundBy << ",with Iter = " << hm.upperboundIter << "\n";
				//return 0;
			}

			std::cout << "Increasing II to " << II << "\n";
		}
		else
		{
			hm.sanityCheck();
			//hm.assignLiveInOutAddr(&tempDFG);
			std::cout << "MAPPING_RESULT :: success II=" << II <<"\n";
			if(PEType == "HyCUBE_4REG"){
				std::cout << "Printing HyCUBE Binary...\n";
#ifdef DUAL_STREAMING
				//hm.printHyCUBEBinary4(tempCGRA);
				hm.printInterconnectData(tempCGRA);
#else
				hm.printHyCUBEBinary(tempCGRA);
#endif
			}
			
		}
	}
	return 0;
}

/*
 * architecture.h
 *
 *      Author: thilini
 */

#ifndef ARCHITECTURE_H_
#define ARCHITECTURE_H_

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "DFG.h"
#include <vector>
#include "PE.h"

using namespace std;
using json = nlohmann::json;

namespace revamp
{

#define NUM_RAND 1
typedef struct{
	int add_count;
	int mul_count;
	int cmp_count;
	int logic_count;
	int mem_count;
}heteroCount;

/*struct myclass {
  bool operator() (const pair<string, int> &a,const pair<string, int> &b){return (a.second < b.second);}
} myobject;
*/


class architecture
{
public:
	int totalsize;
	int xmax;
	int ymax;
	int memPEsize;
	heteroCount heterocount;
	architecture();
	bool ParseJSONArch(std::string fileName);
	bool PreprocessPattern(json &top);
	void calculateComputeRatio(opCount opcount,double effort[4]);
	std::vector<std::pair<string,unsigned int>> distributeOps();
	std::vector<PE*> getPossiblePE(std::string compute);
	int nCr(int n, int r);
	long fact(int n);
	void combinationUtil(std::vector<PE*> arr, PE* data[],int start, int end,int index, int r);
	void printCombination(std::vector<PE*> arr, int n, int r, std::string type);
	std::vector<PE*> PEArr;
	std::vector<std::vector<PE*>> combinations_t;
	void updatePE();
	std::vector<std::pair<std::string,std::string>> computeType;
	std::vector<std::pair<std::string,std::vector<std::string>>> computeFUType;
	void printRTLConfig(int index, bool is_network, bool is_compute, bool is_config,std::string opti);
	void clear();
	bool sortByVal(const pair<string, int> &a,const pair<string, int> &b);

private:
	std::map<string,std::vector<std::vector<PE*>>> combinations;

};
}


#endif /* ARCHITECTURE_H_ */

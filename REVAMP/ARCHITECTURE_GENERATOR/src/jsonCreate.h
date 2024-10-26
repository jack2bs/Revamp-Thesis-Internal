/*
 * json.h
 *
 *      Author: thilini
 */

#ifndef JSONCREATE_H_
#define JSONCREATE_H_
#include <nlohmann/json.hpp>
#include <iostream>
using namespace std;
using json = nlohmann::json;
#include "PE.h"
#include "architecture.h"
namespace revamp
{

class jsonCreate
{
public:
	jsonCreate();
	void extractJson(std::string fileName);
	json getStreamingDP(json &generated);
	json getStreamingFU(json &top,json &generated,string type);
	json getHeteroFU(json &top,json &generated,string type,std::vector<std::string> supportedCompute,bool is_network,bool is_compute);
	json getHeteroPE(json &top,json &generated,string type,string fuType,bool is_network,bool is_compute,bool is_config);
	json getStreamingPE(json &top,json &generated,string type);
	json getHeteroCGRA(json &top,json &generated,std::vector<PE*> peArr,bool is_compute);
	void printJson(architecture *arch,std::string fileName,int index, bool is_network,bool is_compute,bool is_config,std::string opti);
	json dp;
	json mdp;
	json fu;
	json fu_mem;
	json pe;
	json pe_mem;
	json cgra;
private:


};
}



#endif /* JSONCREATE_H_ */

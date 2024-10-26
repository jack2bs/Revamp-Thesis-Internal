/*
 * PE.h
 *
 *      Author: thilini
 */

#ifndef PE_H_
#define PE_H_

#include <string>
#include <vector>
#include <set>
#include <stack>
#include <unordered_set>
#include <unordered_map>

namespace revamp
{

class PE
{
public:
	PE();
	std::vector<std::string> common {"MOVC","CMERGE"};
	std::vector<std::string> adder {"ADD","SUB"};
	std::vector<std::string> logic {"LS","RS","ARS","AND","OR","XOR"};
	std::vector<std::string> comparator {"CMP","CLT","CGT","BR","SELECT"};
	std::vector<std::string> multiplier {"MUL"};
	std::vector<std::string> memory {"LOAD","STORE","LOADB","STOREB","OLOAD","OSTORE"};
	bool is_mem;
	int hotspot_index;
	int x;
	int y;
	std::vector<std::string> supportedCompute;
	std::vector<std::string> supportedOps;
	std::string peType;
	std::string fuType;
	int enable_size=32;
	int router_size=8;
	int opcode_size=4;
	int const_size=4;
	int local_size=4;
private:

};
}




#endif /* PE_H_ */

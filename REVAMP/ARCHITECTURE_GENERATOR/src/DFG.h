/*
 * DFG.h
 *
 *      Author: thilini
 */

#ifndef DFG_H_
#define DFG_H_

#include <string>
#include <vector>
#include <set>
#include <stack>
#include <unordered_set>
#include <unordered_map>

namespace revamp
{
typedef struct{
	int add_count;
	int mul_count;
	int cmp_count;
	int logic_count;
	int mem_count;
	int common_count;

}opCount;

class DFG
{
public:
	std::vector<std::string> common {"MOVC","CMERGE"};
	std::vector<std::string> adder {"ADD","SUB"};
	std::vector<std::string> logic {"LS","RS","ARS","AND","OR","XOR"};
	std::vector<std::string> comparator {"CMP","CLT","CGT","BR","SELECT"};
	std::vector<std::string> multiplier {"MUL"};
	std::vector<std::string> memory {"LOAD","STORE","LOADB","STOREB","OLOAD","OSTORE"};
	opCount opcount;
	DFG();
	bool parseXML(std::string fileName);
	void printDFG();
	std::vector<std::string> nodeOpList;
	opCount analyzeOps();
	void initialize();
private:

};
}


#endif /* DFG_H_ */

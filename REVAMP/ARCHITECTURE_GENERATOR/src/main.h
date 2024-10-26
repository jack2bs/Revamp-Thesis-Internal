/*
 * main.h
 *
 *      Author: thilini
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <vector>
#include <set>
#include <stack>
#include <unordered_set>
#include <unordered_map>
#include "architecture.h"

namespace revamp
{

class main
{
public:
	main();
	bool parseConfig(std::string fileName,architecture* archi, std::vector<PE*> PEArr);
private:

};
}




#endif /* MAIN_H_ */

/*
 * algorithm.cpp
 *
 *      Author: thilini
 */
#include "algorithm.h"
#include <bits/stdc++.h>
#include <iostream>
using namespace std;

namespace revamp
{
algorithm::algorithm()
{
	// TODO Auto-generated constructor stub
}

int algorithm::nCr(int n, int r)
{
    return fact(n) / (fact(r) * fact(n - r));
}

int algorithm::fact(int n)
{
    int res = 1;
    for (int i = 2; i <= n; i++)
        res = res * i;
    return res;
}
}


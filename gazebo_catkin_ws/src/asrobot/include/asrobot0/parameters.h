#ifndef PARAMETERS_H
#define PARAMETERS_H
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <map>

using namespace std;

class Parameters
{
public:
    Parameters();
    Parameters(string filename);

    ~Parameters();

    string getData(string key);

public:
    map<string, string> data;




};

#endif // PARAMETERS_H

#include "../include/stereovision/parameters.h"

Parameters::Parameters()
{
    string filename = "./parameters.txt";
    ifstream fin(filename.c_str());
    if(!fin){
        cerr << "Parameter file does not exist." << endl;
        return;
    }
    while(!fin.eof()){
        string str;
        std::getline(fin, str);
        if('#' == str[0]){
            // '#' is Comments
            continue;
        }
        int pos = str.find("=");
        if(pos == -1){
            continue;
        }
        string key = str.substr(0, pos);
        string value = str.substr(pos+1, str.length());
        data[key] = value;

        if(!fin.good())
            break;
    }
}

Parameters::Parameters(string filename)
{
    ifstream fin(filename.c_str());
    if(!fin){
        cerr << "Parameter file does not exist." << endl;
        return;
    }
    while(!fin.eof()){
        string str;
        std::getline(fin, str);
        if('#' == str.at(0)){
            // '#' is Comments

            continue;
        }

        int pos = str.find("=");
        if(pos == -1){
            continue;
        }
        string key = str.substr(0, pos);
        string value = str.substr(pos+1, str.length());
        data[key] = value;

        if(!fin.good())
            break;
    }
}

Parameters::~Parameters()
{

}

string Parameters::getData(string key)
{
    map<string, string>::iterator iter = data.find(key);
    if(iter == data.end()){
        cerr << "Parameter name " << key << "not found!" << endl;
        return string("NOT_FOUND");
    }
    return iter->second;
}
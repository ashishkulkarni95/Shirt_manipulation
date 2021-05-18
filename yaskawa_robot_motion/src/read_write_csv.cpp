#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>
#include "read_write_csv.hpp"
using std::string;
using std::vector;
using std::ifstream;
using std::ofstream;
using std::istringstream;


// reads a csv of doubles into a vector of vector<double>
vector< vector<double> > read_csv(string filename) {

    vector<vector<double> > values;
    vector<double> valueline;
    
    ifstream fin(filename.c_str());
    string item;
    for (string line; getline(fin, line); ) {
        istringstream in(line);

        while (getline(in, item, ',')) {
            valueline.push_back(atof(item.c_str()));
        }

        values.push_back(valueline);
        valueline.clear();
    }

    return values;

}

// writes vector of vector<double> into a csv
void write_csv(string filename, vector<vector<double> > values) {
    ofstream outputFile(filename, ofstream::out);
    for (auto && valueline : values) {
        for (auto && state : valueline) {
            outputFile << state << ",";
        }
        outputFile << "\n";
    }
    // outputFile << "TEST";
    outputFile.close();
}
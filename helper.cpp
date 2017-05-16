#include "helper.h"
#include <limits>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>



string NextFile(const string baseName, const int digits, const string extension)
{
	//Create a unique filename string based on baseName e.g. abc002 if abc001 exists already
	ifstream theFile;
	int num = 1;
	string filename;
	std::ostringstream  out;
	
	out << setfill('0');
	out << baseName << setw(digits) << num << "." << extension;
	filename = out.str();
		
	theFile.open(filename.c_str() );	
	
	while ( theFile.good() )
	{
		num++;
		
		out.clear();
		out.str("");
		
		out << baseName << setw(digits) << num << "." << extension;
		filename = out.str();
		
		theFile.close();
		theFile.open(filename.c_str() );
		
	}
	
	return filename;
	
}

#include<iostream>
#include<string>
#include<fstream>
#include<sstream>
#include<vector>
using namespace std;


int main(int argC, char **argv) {
/*
	vector<string> inst_arr;
	ifstream infile;
	infile.open(argv[1], ios::in);
	
	int i = 0;
       string instruction;
        while (!infile.eof()){
            getline(infile, instruction);
            inst_arr.push_back(instruction);
            i++;
        }
       
       for (int i = 0; i < inst_arr.size(); i++){
       		cout << inst_arr[i] << endl;
       } 
 
 
 2D vector
 {
 {lw, r1, 0(r0)},
 {add, r2, r1, r0}
 
 }
 
 */
 
 
 
 
 /*** START READ instructions ***/
		 vector <vector <string>> my_inst;
  		 ifstream instruction_file; //keywords
			instruction_file.open(argv[1]);
		  while (instruction_file)
		  {
			string s;
			if (!getline( instruction_file, s )) break;

			stringstream ss(s);
			vector <string> inst_params;

			while (ss)
			{
			  string s;
			  if (!getline( ss, s, ' ' )) break;
			  inst_params.push_back(s);
			}

			my_inst.push_back(inst_params);
		  }
		  if (!instruction_file.eof())
		  {
			cerr << "\n";
		  }
		  
		  for (int i = 0; i < my_inst.size(); i++){
		  	for (int j = 0; j < my_inst[i].size(); j++) {
		  		cout << my_inst[i][j] << " ";
		  	}
		  	cout << endl;
		  }
	
	
	int cycles_counter = 1;
	
		  
	/*** END READ instructions ***/
  	
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 	
       
}

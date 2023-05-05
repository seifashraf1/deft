#pragma once
#include<iostream>
#include<string>
using namespace std;

class ReservationStation{
private:
	int Id;
	string name;
	bool busy;
	string op;
	int Vj;
	int Vk;
	int Qj;
	int Qk;
	long addr;
	int execution_cycles;
public:
	ReservationStation();
	void setId(int _Id);
	void setName(string _name);
	void setBusy(bool bus);
	void setOp(string Op);
	void setVj(int vj);
	void setVk(int vk);
	void setQj(int qj);
	void setQk(int qk);
	void setAddr(long add);
	int getId();
	string getName();
	bool getBusy();
	string getOp();
	int getVj();
	int getVk();
	int getQj();
	int getQk();
	long getAddr();
	void set_execution_cycles(int _execution_cycles);
	int get_execution_cycles();

};


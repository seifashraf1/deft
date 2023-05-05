#include<iostream>
#include<string>
#include<fstream>
#include<sstream>
#include<vector>
#include<map>
#include<queue>
#include "Instruction.h"
#include "ResStatHandler.h"
#include "ReservationStation.h"



using namespace std;




/*
	Function: Check operands ready
	Issue FP
*/

//needs to be completed witih Qk and Vk
void check_operands_ready(Instruction &inst_arr,ReservationStation &rs_arr, map<string, int> &RegStats, map<string, int> &RegFile){
	
if (inst_arr.get_type() == "add" || inst_arr.get_type() == "sub" || inst_arr.get_type() == "div" || inst_arr.get_type() == "beq" || inst_arr.get_type() == "sw"){
	//j not ready
	if (RegStats[inst_arr.get_rs1_name()] != -1){ //-1 means that the regstats available
			rs_arr.setQj(RegStats[inst_arr.get_rs1_name()]);
	
						
	//j ready
	} else {
			rs_arr.setVj(RegFile[inst_arr.get_rs1_name()]);
			rs_arr.setQj(-1);
			
			
	}
	//set address in case of sw
	if (inst_arr.get_type() == "sw"){
		rs_arr.setAddr(inst_arr.get_imm());
	}
	//k not ready
	if (RegStats[inst_arr.get_rs2_name()] != -1){
			rs_arr.setQk(RegStats[inst_arr.get_rs2_name()]);
	
						
	//k ready
	} else {
			rs_arr.setVk(RegFile[inst_arr.get_rs2_name()]);
			rs_arr.setQk(-1);
			
			
	}
	
	
} else if (inst_arr.get_type() == "addi" || inst_arr.get_type() == "neg" || inst_arr.get_type() == "jalr"){
		if (RegStats[inst_arr.get_rs1_name()] != -1){ //-1 means that the regstats available
			rs_arr.setQj(RegStats[inst_arr.get_rs1_name()]);
	
						
		//j ready
		} else {
			rs_arr.setVj(RegFile[inst_arr.get_rs1_name()]);
			rs_arr.setQj(-1);
			
			
		}
	} else if (inst_arr.get_type() == "lw"){
		if (RegStats[inst_arr.get_rs1_name()] != -1){ //-1 means that the regstats available
			rs_arr.setQj(RegStats[inst_arr.get_rs1_name()]);
				
		//j ready
		} else {
			rs_arr.setVj(RegFile[inst_arr.get_rs1_name()]);
			rs_arr.setQj(-1);
			
			
		}
		//address
		rs_arr.setAddr(inst_arr.get_imm());
	}		

 }


 int not_taken = 0;
int main(int argC, char **argv) {
    ResStatHandler object();
    ReservationStation load1;
    ReservationStation load2;
    ReservationStation store1;
    ReservationStation store2;
    ReservationStation branch;
    ReservationStation jal;
    ReservationStation add1;
    ReservationStation add2;
    ReservationStation div;


	//queue to handle load and store
	queue<Instruction> qI;

	map<string, int> RegStats;
	
    RegStats["r0"] = -1;
    RegStats["r1"] = -1;
    RegStats["r2"] = -1;
    RegStats["r3"] = -1;
    RegStats["r4"] = -1;
    RegStats["r5"] = -1;
    RegStats["r6"] = -1;
    RegStats["r7"] = -1;
    RegStats["r8"] = -1;
    RegStats["r9"] = -1;
    RegStats["r10"] = -1;
    RegStats["r11"] = -1;
    RegStats["r12"] = -1;
    RegStats["r13"] = -1;
    RegStats["r14"] = -1;
    RegStats["r15"] = -1;
    
    
    map<string, int> RegFile;
    
    RegFile["r0"] = 0;
    RegFile["r1"] = 1;
    RegFile["r2"] = 2;
    RegFile["r3"] = 3;
    RegFile["r4"] = 4;
    RegFile["r5"] = 0;
    RegFile["r6"] = 0;
    RegFile["r7"] = 0;
    RegFile["r8"] = 0;
    RegFile["r9"] = 1;
    RegFile["r10"] = 2;
    RegFile["r11"] = 4;
    RegFile["r12"] = 6;
    RegFile["r13"] = 8;
    RegFile["r14"] =12;
    RegFile["r15"] = 32;
    
    map<string, int> inst_type;     //map for the instruction types to a number
    inst_type["add"] = 0;
    inst_type["neg"] = 0;
    inst_type["addi"] = 0;
    inst_type["div"] = 1;
    inst_type["jalr"] = 2;
    inst_type["ret"] = 2;
    inst_type["beq"] = 3;
    inst_type["lw"] = 4;
    inst_type["sw"] = 5;




	vector <vector <string> > my_inst;
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

		  
		//Dynamic array for instruction objects
		Instruction *inst_arr;
		inst_arr = new Instruction[100];
		
		
		int inst_counter = 0; 
		  //Loop to instantiate object for every instruction
		  
		for (int i = 0; i < my_inst.size(); i++){
		   	for (int j = 0; j < my_inst[i].size(); j++) {
		   		Instruction inst = Instruction();
		   		inst.set_type(my_inst[i][0]);
		   		if (inst.get_type() == "add" || inst.get_type() == "sub" || inst.get_type() == "div"){
		   			
		   			//set rd
		   			inst.set_rd_name(my_inst[i][1]);
		   			inst.set_rd(RegFile[my_inst[i][1]]);
		   			
		   			//set rs1
		   			inst.set_rs1_name(my_inst[i][2]);
		   			inst.set_rs1(RegFile[my_inst[i][2]]);
		   			
		   			//set rs2
		   			inst.set_rs2_name(my_inst[i][3]);
		   			inst.set_rs2(RegFile[my_inst[i][3]]);
		   			
		   			inst.set_execution_cycles(2);
		   			inst.set_execution_counter(2);
		   			if (inst.get_type() == "div"){
						inst.set_execution_cycles(8);
		   				inst.set_execution_counter(8);		   				
		   			}
		   			inst_arr[i] = inst;
		   			
		   		
		   			
		   		} else if (inst.get_type() == "addi"){
		   			
		   			
		   			//set rd
		   			inst.set_rd_name(my_inst[i][1]);
		   			inst.set_rd(RegFile[my_inst[i][1]]);
		   			
		   			//set rs1
		   			inst.set_rs1_name(my_inst[i][2]);
		   			inst.set_rs1(RegFile[my_inst[i][2]]);
		   			
		   			//set imm
		   			//convert string to interger
		   			stringstream temp_str(my_inst[i][3]);
		   			int num = 0;
		   			temp_str >> num;
		   			inst.set_imm(num);
		   			
		   			inst.set_execution_cycles(2);
		   			inst.set_execution_counter(2);
		   			inst_arr[i] = inst;		   				   			
		   			
		   		} else if (inst.get_type() == "neg"){
		   			//set rd
		   			inst.set_rd_name(my_inst[i][1]);
		   			inst.set_rd(RegFile[my_inst[i][1]]);
		   			
		   			//set rs1
		   			inst.set_rs1_name(my_inst[i][2]);
		   			inst.set_rs1(RegFile[my_inst[i][2]]);	
		   			
		   			
		   			inst.set_execution_cycles(2);
		   			inst.set_execution_counter(2);
		   			inst_arr[i] = inst;			   			
		   				   			
		   		} else if (inst.get_type() == "beq"){
		   			
		   			
		   			//set rs1
		   			inst.set_rs1_name(my_inst[i][1]);
		   			inst.set_rs1(RegFile[my_inst[i][1]]);	
		   			
		   			//set rs2
		   			inst.set_rs2_name(my_inst[i][2]);
		   			inst.set_rs2(RegFile[my_inst[i][2]]);			   			
		  
		   			//set imm
		   			//convert string to interger
		   			stringstream temp_str(my_inst[i][3]);
		   			int num = 0;
		   			temp_str >> num;
		   			inst.set_imm(num);	  
		   			
		   			//set write cycle
		   			inst.set_write_cycle(-1); 			
		   			
		   			inst.set_execution_cycles(1);
		   			inst.set_execution_counter(1);
		   			inst_arr[i] = inst;			   			
		   				   			
		   		} else if (inst.get_type() == "jalr"){
		   			
		   			
		   			//set rs1
		   			inst.set_rs1_name(my_inst[i][1]);
		   			inst.set_rs1(RegFile[my_inst[i][1]]);	
		   			
		   			
		   			 
		   			
		   			//set write cycle
		   			inst.set_write_cycle(-1); 			
		   			
		   			inst.set_execution_cycles(1);
		   			inst.set_execution_counter(1);
		   			inst_arr[i] = inst;			   			
		   				   			
		   		} else if (inst.get_type() == "ret"){
		   			
		   			
		   		
		   			
		   			//set write cycle
		   			inst.set_write_cycle(-1); 			
		   			
		   			inst.set_execution_cycles(1);
		   			inst.set_execution_counter(1);
		   			inst_arr[i] = inst;			   			
		   				   			
		   		} else if (inst.get_type() == "lw"){
		   			
		   			//set rd
		   			inst.set_rd_name(my_inst[i][1]);
		   			inst.set_rd(RegFile[my_inst[i][1]]);
		   		
		   			//get second element 4(r3)
		   			string str = my_inst[i][2];
		   			//cout << "str: " << str << endl;
		   			string immediate = "";
		   			string my_reg = "";
		   			int imm_counter = 0;
		   			for (int i = 0; i < str.length(); i++){
		   				if (str[i] != '('){
		   		
		   					immediate += str[i];
		   				
		   				} else {
		   					break;
		   				}
		   			}
		   			string temp_rs1;
		   			for (int i = 0; i < str.length(); i++){
		   				if (str[i]=='('){
		   					temp_rs1 = str[i+1];
		   					temp_rs1 += str[i+2];
		   					break; 
		   				}
		   			}
		   			
		   			//set rs1
		   			inst.set_rs1_name(temp_rs1);
		   			inst.set_rs1(RegFile[temp_rs1]);
		   			
		   			//convert immediate string to integer and save it in the imm in instruction
		   			//set imm
		   			//convert string to interger
		   			stringstream temp_str2(immediate);
		   			int num2 = 0;
		   			temp_str2 >> num2;
		   			inst.set_imm(num2);
		   			
		   			//set write cycle
		   			inst.set_write_cycle(-1); 			
		   			
		   			inst.set_execution_cycles(2);
		   			inst.set_execution_counter(2);
		   			inst_arr[i] = inst;			   			
		   				   			
		   		}else if (inst.get_type() == "sw"){
		   			
		   			//set rd
		   			inst.set_rs2_name(my_inst[i][1]);
		   			inst.set_rs2(RegFile[my_inst[i][1]]);
		   		
		   			//get second element 4(r3)
		   			string str = my_inst[i][2];
		   			//cout << "str: " << str << endl;
		   			string immediate = "";
		   			string my_reg = "";
		   			int imm_counter = 0;
		   			for (int i = 0; i < str.length(); i++){
		   				if (str[i] != '('){
		   		
		   					immediate += str[i];
		   				
		   				} else {
		   					break;
		   				}
		   			}
		   			string temp_rs1;
		   			for (int i = 0; i < str.length(); i++){
		   				if (str[i]=='('){
		   					temp_rs1 = str[i+1];
		   					temp_rs1 += str[i+2];
		   					break; 
		   				}
		   			}
		   			
		   			//set rs1
		   			inst.set_rs1_name(temp_rs1);
		   			inst.set_rs1(RegFile[temp_rs1]);
		   			
		   			//convert immediate string to integer and save it in the imm in instruction
		   			//set imm
		   			//convert string to interger
		   			stringstream temp_str2(immediate);
		   			int num2 = 0;
		   			temp_str2 >> num2;
		   			inst.set_imm(num2);
		   		
		   			inst.set_execution_cycles(2);
		   			inst.set_execution_counter(2);
		   			inst_arr[i] = inst;			   			
		   				   			
		   		} 
		   		
		   	}
		   	inst_counter++;
		 
		   }
		   
		

		//Dynamic array for memory
		int *mem;
		mem = new int[64000];   
		for (int i = 0 ; i < 64000; i++){
			mem[i] = 0;
		}
		mem[0] = 10;
		mem[1] = 20;
		   
		   
		Instruction **all_inst = new Instruction *[100];
		for (int i = 0; i < 100; i++){
			all_inst[i] = new Instruction[1];
		}
	
		int all_inst_counter = 0;
		   		  
		//Dynamic array for instruction objects
		Instruction *backend;
		backend = new Instruction[100];
		   
		
		int jalr_stopping_address = 0;

		bool first_time = true;
        int main_clk = 1;
        ResStatHandler RS_handler;
        //Dynamic array for instruction objects
		ReservationStation *rs_arr;
		rs_arr = new ReservationStation[100];
		
	
		int issue_counter = 0;
		int backend_size = 0;
		int temp = 100;
		int taken = 0;
		int total_branches = 0;
        while(temp != 0){
        	//first step
        	
        	for (int i = 0; i < issue_counter; i++){
        	
        		
        		//check type of instruction

        		if (backend[i].get_status() == 1){
        		
        			//before execute
        			
        			if (backend[i].get_type() == "add" || backend[i].get_type() == "sub" || backend[i].get_type() == "div" || backend[i].get_type() == "beq"){
        			
        				
        				if (rs_arr[backend[i].get_rs_id()].getQj() == -1 && rs_arr[backend[i].get_rs_id()].getQk() == -1){
        					
        				//add and sub
        					backend[i].set_status(2); //executing
        					backend[i].set_start_execute_clk(main_clk);
        					backend[i].set_end_execute_clk(main_clk + backend[i].get_execution_cycles() - 1);
        					backend[i].set_rs1(RegFile[backend[i].get_rs1_name()]);
        					backend[i].set_rs2(RegFile[backend[i].get_rs2_name()]);
        					if (backend[i].get_type() == "add") {
        						backend[i].set_rd(backend[i].get_rs1() + backend[i].get_rs2());
        					} else if (backend[i].get_type() == "sub"){
        						backend[i].set_rd(backend[i].get_rs1() - backend[i].get_rs2());
        					} else if (backend[i].get_type() == "div") {
        						if (backend[i].get_rs2() != 0) {
        							backend[i].set_rd(backend[i].get_rs1() / backend[i].get_rs2());
        						}
        					} else if (backend[i].get_type() == "beq") {
        					total_branches += 1;
        						if (backend[i].get_rs1() == backend[i].get_rs2()){
        							taken +=1;
        							issue_counter += backend[i].get_imm();
        							
        						}
        					}
        					
        					
        				}	
        			//addi
        			} else if (backend[i].get_type() == "addi"){
						if (rs_arr[backend[i].get_rs_id()].getQj() == -1) {
        			
        					backend[i].set_status(2); //executing
        					backend[i].set_start_execute_clk(main_clk);
        					backend[i].set_end_execute_clk(main_clk + backend[i].get_execution_cycles() - 1);
        					backend[i].set_rs1(RegFile[backend[i].get_rs1_name()]);		
        					backend[i].set_rd(backend[i].get_rs1() + backend[i].get_imm());					
						}
				}
				//neg
				else if (backend[i].get_type() == "neg"){
						if (rs_arr[backend[i].get_rs_id()].getQj() == -1) {
        			
        					backend[i].set_status(2); //executing
        					backend[i].set_start_execute_clk(main_clk);
        					backend[i].set_end_execute_clk(main_clk + backend[i].get_execution_cycles() - 1);
        					backend[i].set_rs1(RegFile[backend[i].get_rs1_name()]);		
        					backend[i].set_rd(backend[i].get_rs1() * -1);					
						}
				}	//jalr
				else if (backend[i].get_type() == "jalr"){
						if (rs_arr[backend[i].get_rs_id()].getQj() == -1) {
        				
        					backend[i].set_status(4); //executing
        					RS_handler.incrementJal();
        					backend[i].set_start_execute_clk(main_clk);
        					backend[i].set_end_execute_clk(main_clk + backend[i].get_execution_cycles() - 1);
        					backend[i].set_rs1(RegFile[backend[i].get_rs1_name()]);		
        					RegFile["r1"] = issue_counter; //save Pc + 1 in r1
        					issue_counter = issue_counter -1 + backend[i].get_rs1(); //set address
        					jalr_stopping_address = issue_counter;
        					
 						}
				} //ret
				else if (backend[i].get_type() == "ret" && first_time){
						first_time = false;
        			
        					backend[i].set_status(4); //executing
        					
        					RS_handler.incrementJal();
        					backend[i].set_start_execute_clk(main_clk);
        					backend[i].set_end_execute_clk(main_clk + backend[i].get_execution_cycles() - 1);
        					issue_counter = RegFile["r1"]; //set PC with value in r1
	
				}//load
				else if (backend[i].get_type() == "lw"){
					if (backend[i].get_issue_clk() == qI.front().get_issue_clk()){
        					backend[i].set_status(2); //executing
        					backend[i].set_start_execute_clk(main_clk);
        					backend[i].set_end_execute_clk(main_clk + backend[i].get_execution_cycles() -1);
        					rs_arr[backend[i].get_rs_id()].setAddr(rs_arr[backend[i].get_rs_id()].getAddr() + backend[i].get_rs1());

        				}
					
				}//store
				else if (backend[i].get_type() == "sw"){
					if (backend[i].get_issue_clk() == qI.front().get_issue_clk()){
        					backend[i].set_status(2); //executing
        					backend[i].set_start_execute_clk(main_clk);
        					backend[i].set_end_execute_clk(main_clk + backend[i].get_execution_cycles() -1);
        					rs_arr[backend[i].get_rs_id()].setAddr(rs_arr[backend[i].get_rs_id()].getAddr() + backend[i].get_rs1());
        				
	
					}
				}
				
				
				
	
				
        			//executing stage
        		} else if (backend[i].get_status() == 2) {
        			backend[i].decrement_execution_counter(); //decrement execution cycles
        			if (backend[i].get_execution_counter() == 0){
        			//finish executing
        				if (backend[i].get_type() == "beq"){
        					RS_handler.incrementBeq();
        					backend[i].set_status(4);
        					
        				} else if (backend[i].get_type() == "lw") {
        					RS_handler.incrementLoad();
        					backend[i].set_status(4);
        					RegFile[backend[i].get_rd_name()] = mem[rs_arr[backend[i].get_rs_id()].getAddr()];  
        					qI.pop();     
        						
        					    for (int k = 0; k < issue_counter; k++){        			
        						if (rs_arr[backend[k].get_rs_id()].getQj()== backend[i].get_rs_id()) {        			
        							rs_arr[backend[k].get_rs_id()].setVj(backend[i].get_rd());
        							rs_arr[backend[k].get_rs_id()].setQj(-1);
        						}
        						if (rs_arr[backend[k].get_rs_id()].getQk() == backend[i].get_rs_id()) {
        							rs_arr[backend[k].get_rs_id()].setVk(backend[i].get_rd());
        							rs_arr[backend[k].get_rs_id()].setQk(-1);
        				
        						} 
        					}	
        						
        				} 
        				
        				else {
        					backend[i].set_status(3);
        				
        				}
        				
        			
        				
        			}
        			//write stage
        		} else if (backend[i].get_status() == 3){
        			backend[i].set_write_cycle(main_clk - 1);
        			if (backend[i].get_type() == "add" || backend[i].get_type() == "sub" || backend[i].get_type() == "neg" || backend[i].get_type() == "addi"){
        				RS_handler.incrementAdd();
        				RegFile[backend[i].get_rd_name()] = backend[i].get_rd();
        				if (backend[i].get_type() == "addi"){
        					RegFile[backend[i].get_rd_name()] = backend[i].get_rd();
        				}
        			} else if (backend[i].get_type() == "sw") {
        				RS_handler.incrementStore();
        				
        				mem[rs_arr[backend[i].get_rs_id()].getAddr()] = RegFile[backend[i].get_rs2_name()]; 
        				qI.pop();
        					
        			} 
        			
          			RegStats[backend[i].get_rd_name()] = -1;
        			backend[i].set_status(4);
        			for (int k = 0; k < issue_counter; k++){
        				if (backend[k].get_type() == "add" || backend[k].get_type() == "sub" || backend[k].get_type() == "div"){
        				if (rs_arr[backend[k].get_rs_id()].getQj()== backend[i].get_rs_id()) {	
        					rs_arr[backend[k].get_rs_id()].setVj(backend[i].get_rd());
        					rs_arr[backend[k].get_rs_id()].setQj(-1);
        				}
        				
        				if (rs_arr[backend[k].get_rs_id()].getQk() == backend[i].get_rs_id()) {
        					rs_arr[backend[k].get_rs_id()].setVk(backend[i].get_rd());
        					rs_arr[backend[k].get_rs_id()].setQk(-1);
        				
        				}  
        				} else if (backend[k].get_type() == "addi" || backend[k].get_type() == "neg"){
         				if (rs_arr[backend[k].get_rs_id()].getQj()== backend[i].get_rs_id()) {   					
        					rs_arr[backend[k].get_rs_id()].setVj(backend[i].get_rd());
        					rs_arr[backend[k].get_rs_id()].setQj(-1);
        				}       					
        				}     				
        			}
        		
        		}
        	}
        	/*
        	bool cont_flag = false; //false means will break infinite loop
        	        	if (backend_size == 0) {
        		cont_flag = true; //special case for first time
        	}
        	for (int i = 0; i < backend_size; i++){
        		if (backend[i].get_status() != 4){
        			cont_flag = true;
        		}
        	}
        	*/
        	/*
        	bool cont_flag = false; //false means will break infinite loop
        	//logic to break infinte loop

        	
        	for (int i = 0; i < backend_size; i++){
        		if (backend[i].get_write_cycle() == 0){
        			cont_flag = true;
        		}
        	}
        	
        	
        	*/
        	
 
       	/*
       		print trial
       	*/
    	
        	//second step [issue a new instruction]
        	string my_type = inst_arr[issue_counter].get_type(); //check
        	inst_arr[issue_counter].set_status(4);
        	bool check_avail;
        	if (my_type == "add" || my_type == "sub" || my_type == "addi" || my_type == "neg" || my_type == "div" || my_type == "beq" || my_type == "jalr" || my_type == "ret" || my_type == "lw" || my_type == "sw"){
        		//check reservation station
        		
        		if (my_type == "add" || my_type == "sub" || my_type == "addi" || my_type == "neg"){
        			check_avail = RS_handler.is_add_available();
        		} else if (my_type == "div"){
        			check_avail = RS_handler.is_div_available();
        		} else if (my_type == "beq"){
        			check_avail = RS_handler.is_beq_available();
        		} else if (my_type == "jalr" || my_type == "ret"){
        			check_avail = RS_handler.is_jal_available();
        		}else if (my_type == "lw"){
        			check_avail = RS_handler.is_load_available();
        		}else if (my_type == "sw"){
        			check_avail = RS_handler.is_store_available();
        		}
        		
        		
        		if (check_avail){
        			if (!first_time){
        				if (issue_counter < jalr_stopping_address){
        					inst_arr[issue_counter].set_status(1);
        					inst_arr[issue_counter].set_issue_clk(main_clk);
		  					if (my_type == "add" || my_type == "sub" || my_type == "addi" || my_type == "neg"){
		  						RS_handler.decrementAdd(); //decrement add RS
		  					} else if (my_type == "div"){
		  						RS_handler.decrementDiv(); 
		  					} else if (my_type == "beq"){
		  						not_taken++;
		  						RS_handler.decrementBeq(); 
		  					} else if (my_type == "jalr" || my_type == "ret"){
		  						RS_handler.decrementJal(); 
		  					} else if (my_type == "lw"){
  								RS_handler.decrementLoad(); 
  							} else if (my_type == "sw"){
  								RS_handler.decrementStore(); 
  							}     			
        			
							
							
							ReservationStation rs = ReservationStation();
							rs_arr[issue_counter] = rs;
							inst_arr[issue_counter].set_rs_id(issue_counter);
							backend[issue_counter] = inst_arr[issue_counter]; //append the issued instrucion to the backend array
							backend_size++;
							
					        all_inst[all_inst_counter] = &(backend[issue_counter]);
        					all_inst_counter++; 
							//check operand ready and do some creepy things after issuing
							
							check_operands_ready(inst_arr[issue_counter], rs_arr[issue_counter], RegStats,RegFile);
							
							RegStats[backend[issue_counter].get_rd_name()] = backend[issue_counter].get_rs_id();
		        			if (backend[issue_counter].get_type() == "lw" || backend[issue_counter].get_type() == "sw"){
        						qI.push(backend[issue_counter]);
        					}
							issue_counter++; //This will act as the PC address
        				}
        			} else {
        			inst_arr[issue_counter].set_status(1);
        			inst_arr[issue_counter].set_issue_clk(main_clk);
  					if (my_type == "add" || my_type == "sub" || my_type == "addi" || my_type == "neg"){
  						RS_handler.decrementAdd(); //decrement add RS
  					} else if (my_type == "div"){
  						RS_handler.decrementDiv(); 
  					} else if (my_type == "beq"){
  						not_taken++;
  						RS_handler.decrementBeq(); 
  					} else if (my_type == "jalr" || my_type == "ret"){
  						RS_handler.decrementJal(); 
  					} else if (my_type == "lw"){
  						RS_handler.decrementLoad(); 
  					} else if (my_type == "sw"){
  						RS_handler.decrementStore(); 
  					}     			
        			
        			ReservationStation rs = ReservationStation();
        			rs_arr[issue_counter] = rs;
        			inst_arr[issue_counter].set_rs_id(issue_counter);
        			backend[issue_counter] = inst_arr[issue_counter]; //append the issued instrucion to the backend array
        			all_inst[all_inst_counter] = &(backend[issue_counter]);
        			all_inst_counter++; 
        			backend_size++;
        			
        	
        			//check operand ready and do some creepy things after issuing
        			
        			check_operands_ready(inst_arr[issue_counter], rs_arr[issue_counter], RegStats,RegFile);
        			
        			RegStats[backend[issue_counter].get_rd_name()] = backend[issue_counter].get_rs_id();
        			if (backend[issue_counter].get_type() == "lw" || backend[issue_counter].get_type() == "sw"){
        				qI.push(backend[issue_counter]);
        			}
        			/*cout << endl;
        			backend[issue_counter].print_info();
        			cout << endl;*/
        			issue_counter++; //This will act as the PC address
        			
        			}
        		}
    
      
        	} 
        	main_clk++;
        	temp--;
        
        /*
        
        	if (!cont_flag){
        	cout << "break;"<< endl;
        		break;
        	}
        	
*/
        
        }
        
        //test
        
        for (int i = 0; i < all_inst_counter; i++){
        if (all_inst[i]->get_type() == "") continue;
        cout << "Instruction: " << i + 1 << endl;      
        	all_inst[i]->print_info();
        	cout << endl;
        }
        
        
        /* Calculate total cycles */
        
        int max = 0;
        for (int i = 0; i < issue_counter; i++){
        	if (backend[i].get_write_cycle() > max){
        		max = backend[i].get_write_cycle();
        	}
        }

        cout << "The number of total cycles to finish the program: " << max << endl;
	
	
        /* IPC */
        float IPC = (all_inst_counter *1.0) / (max *1.0) ;
        cout << "Instructions per clock: " << IPC << endl;
        
        /* Branch misprediction */
       
        
        float misprediction_ratio=0;
        if (total_branches != 0){
        	misprediction_ratio= (not_taken * 1.0) / (total_branches * 1.0);
        }
        cout << "Misprediction ratio: " << misprediction_ratio << endl;
        /*
        cout << "Reg r2 " << RegFile["r2"] << endl;
        cout << "Reg r10 " << RegFile["r10"] << endl;
        cout << "Reg r11 " << RegFile["r11"] << endl;
        */
        
        

 
}


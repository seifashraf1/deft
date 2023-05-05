#include "Instruction.h"


Instruction::Instruction(){
	status = 0;
	rd = 0;
	rs1 = 0;
	rs2 = 0;
	rs1_name = "";
	rs2_name = "";
	rd_name = "";
	issue_clk = 0;
	start_execute_clk = 0;
	end_execute_clk = 0;
	write_cycle = 0;
	imm = 0;
	
}

//get reservation station
 ReservationStation Instruction::get_reservation_station(){
       return rs;
}

 void Instruction::setClk(int c){
       clk = c;
    }
  
  
 void Instruction::set_imm(int _imm){
       imm = _imm;
    }      
 int Instruction::getClk(){return clk;}
 
 
 //set rd
 void Instruction::set_rd(int _rd){
 	rd = _rd;
 }
 
 //set rs1
 void Instruction::set_rs1(int _rs1){
 	rs1 = _rs1;
 }
 
 
 //set rs2
 void Instruction::set_rs2(int _rs2){
 	rs2 = _rs2;
 }
 
 //set name
 void Instruction::set_name(string _name){
 	name = _name;
 }
 
 //set status
  void Instruction::set_status(int _status){
 	status = _status;
 }
 
  
 //set issue clk
  void Instruction::set_issue_clk(int _issue_clk){
 	issue_clk = _issue_clk;
 }
 
 
   
 //set rs_id
  void Instruction::set_rs_id(int _id){
 	rs_id = _id;
 }
 
 
 //get rd
 int Instruction::get_rd(){
 	return rd;
 }
 
 //get rs1
 int Instruction::get_rs1(){
 	return rs1;
 }
 
 //get rs2
 int Instruction::get_rs2(){
 	return rs2;
 }
 
  //get name
 string Instruction::get_name(){
 	return name;
 }
 
 //get status
 int Instruction::get_status(){
 	return status;
 }
 

  
 //get imm
 int Instruction::get_imm(){
 	return imm;
 }
  
 //get issue lck
 int Instruction::get_issue_clk(){
 	return issue_clk;
 }
 
 
 //set rd name
 void Instruction::set_rd_name(string _rd_name){
 	rd_name = _rd_name;
 }
 //set rs1 name
  void Instruction::set_rs1_name(string _rs1_name){
 	rs1_name = _rs1_name;
 }
 //set rs2 name
  void Instruction::set_rs2_name(string _rs2_name){
 	rs2_name = _rs2_name;
 }
 
 //set type
 void Instruction::set_type(string _type){
 	type = _type;
 }
 
 //get rd name
  string Instruction::get_rd_name(){
 	return rd_name;
 }
 
 
 //get rs1 name
  string Instruction::get_rs1_name(){
 	return rs1_name;
 }
 
 
 //get rs2 name
  string Instruction::get_rs2_name(){
 	return rs2_name;
 }
 
 int Instruction::get_start_execute_clk(){
 	return start_execute_clk;
 }
 
  int Instruction::get_end_execute_clk(){
 	return end_execute_clk;
 }
 
  int Instruction::get_execution_cycles(){
 	return execution_cycles;
 }
 
  int Instruction::get_execution_counter(){
 	return execution_counter;
 }
 

 
  int Instruction::get_write_cycle(){
 	return write_cycle;
 }

 string Instruction::get_type(){
 	return type;
 }
 
 
 
 int Instruction::get_rs_id(){
 	return rs_id;
 }
 
 void Instruction::print_clk(){
 	cout << "issue clk: " << get_issue_clk() << endl;
 }
 
 void Instruction::set_reservation_station(ReservationStation _rs){
 	rs = _rs;
 }
 
 void Instruction::set_start_execute_clk(int c){
 	start_execute_clk = c;
 }
 
  void Instruction::set_end_execute_clk(int c){
 	end_execute_clk = c;
 }
 
 
void Instruction::set_execution_cycles(int c){
 	execution_cycles = c;
 }
 
 void Instruction::set_execution_counter(int c){
 	execution_counter = c;
 }
 
 
  void Instruction::set_write_cycle(int c){
 	write_cycle = c;
 }

 void Instruction::decrement_execution_counter(){
 	execution_counter--;
 }
 
 void Instruction::print_info(){
 	cout << "Type: " << type << endl;
 	//cout << "rs1: " << rs1_name << " " << rs1 << endl;
 	//cout << "rs2: " << rs2_name << " " << rs2 << endl;
 	//cout << "rd: " << rd_name << " " << rd << endl;
 	cout << "issue clock: " << issue_clk << endl;
 	cout << "start execute clock: " << start_execute_clk<< endl;
 	cout << "end execute clock: " << end_execute_clk << endl;
 	cout << "write clock: " << write_cycle << endl;
 	
 	
 }
 
 
 
 
 
 
 
 
 
 
 

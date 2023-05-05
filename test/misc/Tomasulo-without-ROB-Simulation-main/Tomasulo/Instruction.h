#pragma once
#include<iostream>
using namespace std;
#include "ReservationStation.h"

class Instruction{
private:
    int numberOfInstructions;
    int clk;
    string name;
    string rs1_name;
    string rs2_name;
    string rd_name;
    int rs1;
    int rs2;
    int rd; 
    int status;
    int issue_clk;
    int start_execute_clk;
    int end_execute_clk;
    ReservationStation rs;
    string type;
    int rs_id;
    int execution_cycles;
    int execution_counter;
    int write_cycle;
    int imm;
    
public:
    Instruction();
    void setClk(int c);
    int getClk();
    int get_rd();
    int get_rs1();
    int get_rs2();
    int get_imm();
    
    void set_type(string _type);
    
    string get_rd_name();
    string get_rs1_name();
    string get_rs2_name();
    
    int get_status();
    int get_issue_clk();
    string get_name();
    string get_type();
    
    ReservationStation get_reservation_station();
    void set_rs_id(int);
    int get_rs_id();
    int get_start_execute_clk();
    int get_end_execute_clk();
    int get_execution_cycles();
    int get_execution_counter();
    int get_write_cycle();
    
    void set_write_cycle(int);
    void set_execution_cycles(int);
    void set_execution_counter(int);
    void set_rd(int _rd);
    void set_rs1(int _rs1);
    void set_rs2(int _rs2);
    void set_name(string _name);
    void set_status(int _status);
    void set_issue_clk(int _issue_clk);
    void set_start_execute_clk(int);
    void set_end_execute_clk(int);
    void set_rd_name(string _rd_name);
    void set_rs1_name(string _rs1_name);
    void set_rs2_name(string _rs2_name);
    void set_imm(int _imm);
    void decrement_execution_counter();
    
    void set_reservation_station(ReservationStation _rs);
    void print_clk();
    
    void print_info();
};

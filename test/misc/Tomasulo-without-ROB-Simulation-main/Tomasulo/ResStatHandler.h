#pragma once
#include<iostream>
using namespace std;


class ResStatHandler{
private:
    int LoadNum;
    int StoreNum;
    int AddNum;
    int DivNum;
    int JalNum;
    int BeqNum;
public:
    ResStatHandler();
    void incrementLoad();
    void incrementStore();
    void incrementAdd();
    void incrementDiv();
    void incrementJal();
    void incrementBeq();


    void decrementLoad();
    void decrementStore();
    void decrementAdd();
    void decrementDiv();
    void decrementJal();
    void decrementBeq();

    bool is_load_available();

    bool is_store_available();

    bool is_add_available();

    bool is_beq_available();

    bool is_div_available();
    bool is_jal_available();
};


